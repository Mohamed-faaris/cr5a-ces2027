"""PGEA transport through dobot_bringup_v4's ROS Modbus services.

Why this exists
---------------
The CR controller serves exactly one Dashboard TCP client (:29999) as command
owner, and dobot_bringup_v4_node holds that connection for its whole lifetime
(command.cpp: TcpClient(ip, 29999)). A second Dashboard client — what
transport:=dashboard creates — completes the TCP handshake but the controller
drops it on the first command ("Dobot closed the Dashboard socket"), which is
the "opens but no response" failure of this bridge inside the full stack.

This transport routes the *identical* Dashboard command strings through the
arm driver's ROS services instead (they travel over its own connection):

    SetTool485   -> /dobot_bringup_ros2/srv/SetTool485
    ModbusCreate -> /dobot_bringup_ros2/srv/ModbusCreate
    GetHoldRegs  -> /dobot_bringup_ros2/srv/GetHoldRegs
    SetHoldRegs  -> /dobot_bringup_ros2/srv/SetHoldRegs
    ModbusClose  -> /dobot_bringup_ros2/srv/ModbusClose

Cross-checked against bringup's parseTool.cpp and the vendor V4 API doc
(V4新增指令/README.md §3.32/§3.55):

* ip is passed WITH embedded quotes, so bringup's raw field streaming
  reproduces ModbusCreate("127.0.0.1",60000,<slave>,1) byte-for-byte — the
  exact form the vendor docs and the proven standalone app use (isRTU=1
  selects the RTU master on the 60000 tool-RS485 passthrough port),
* SetTool485 officially takes only the baudrate (vendor: 115200 => no parity,
  stop bit 1 — exactly the PGEA default; parity/stopbits params are unused in
  this mode),
* the controller supports masters index 0..4 (vendor: max 5 devices).

Index discovery
---------------
Bringup's ModbusCreate callback discards the returned {index} (only res is
filled), so the session index is found by harmless reads of register 0x0200:

1. probe existing masters 0..4 first — adopts a live session left by a
   crashed previous run instead of leaking a new one (vendor cap: 5),
2. if none answers, ModbusCreate and probe again to find our own,
3. if creation succeeded but nothing answers (gripper unplugged), release all
   masters 0..4 before failing, so retries can never exhaust the cap.

Only this stack creates masters on this controller, and sessions bind the
Modbus slave at creation time — a reused session therefore assumes the same
slave_id it was created with (restart the controller after changing it).

Requires dobot_bringup_v4_node running; without it every call fails with a
clear "service unavailable" error and the bridge keeps retrying.
"""
from __future__ import annotations

import threading
import time
from typing import Optional

from rclpy.callback_groups import ReentrantCallbackGroup

from dobot_msgs_v4.srv import (
    GetHoldRegs,
    ModbusClose,
    ModbusCreate,
    SetHoldRegs,
    SetTool485,
)

from pgea_gripper.pgea_app.transport import TransportError

_SERVICE_NS = "/dobot_bringup_ros2/srv/"
_PROBE_INDICES = range(5)  # vendor: max 5 modbus masters, index 0..4


def _parse_u16_values(raw: str, count: int, context: str) -> list[int]:
    """Parse a GetHoldRegs robot_return such as "{13}" or "{1,2,3}"."""
    content = (raw or "").strip()
    if content.startswith("{") and content.endswith("}"):
        content = content[1:-1].strip()
    items = [item.strip() for item in content.split(",")] if content else []
    if len(items) != count:
        raise TransportError(
            f"{context}: expected {count} register value(s), got {raw!r}"
        )
    values: list[int] = []
    for item in items:
        try:
            value = int(item, 0)
        except ValueError:
            try:
                value = int(item, 10)
            except ValueError:
                raise TransportError(
                    f"{context}: non-integer register reply {raw!r}"
                ) from None
        if not 0 <= value <= 0xFFFF:
            raise TransportError(
                f"{context}: register value outside unsigned 16-bit range: {raw!r}"
            )
        values.append(value)
    return values


class BringupServiceTransport:
    """PGEA Modbus registers via dobot_bringup_v4 ROS services.

    Same duck-typed interface as pgea_app.transport.DobotToolRS485Transport
    (connected/connect/close/read_registers/read_register/write_register/
    last_tx/last_rx) so PGEADriver and the bridge node use it unchanged.
    """

    def __init__(
        self,
        node,
        slave_id: int = 1,
        baudrate: int = 115200,
        parity: str = "N",
        stopbits: int = 1,
        timeout: float = 2.5,
    ):
        # Same validation as the vendored dashboard transport.
        if not 1 <= slave_id <= 247:
            raise ValueError("PGEA Modbus slave ID must be 1-247")
        if baudrate not in (4800, 9600, 19200, 38400, 57600, 115200):
            raise ValueError("Unsupported PGEA baud rate")
        if parity not in ("N", "E", "O"):
            raise ValueError("Parity must be N, E or O")
        if stopbits not in (1, 2):
            raise ValueError("Stop bits must be 1 or 2")
        self.slave_id = slave_id
        self.baudrate = baudrate
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = float(timeout)

        self._node = node
        self._lock = threading.RLock()
        self._connected = False
        self.modbus_index: Optional[int] = None
        self.last_tx = ""
        self.last_rx = ""

        # Own reentrant group: service responses must be processable by
        # another MultiThreadedExecutor worker even while one of our threads
        # waits on a future (never block the executor on itself).
        self._cbg = ReentrantCallbackGroup()
        self._srv_485 = node.create_client(
            SetTool485, _SERVICE_NS + "SetTool485", callback_group=self._cbg
        )
        self._srv_create = node.create_client(
            ModbusCreate, _SERVICE_NS + "ModbusCreate", callback_group=self._cbg
        )
        self._srv_close = node.create_client(
            ModbusClose, _SERVICE_NS + "ModbusClose", callback_group=self._cbg
        )
        self._srv_get = node.create_client(
            GetHoldRegs, _SERVICE_NS + "GetHoldRegs", callback_group=self._cbg
        )
        self._srv_set = node.create_client(
            SetHoldRegs, _SERVICE_NS + "SetHoldRegs", callback_group=self._cbg
        )

    # ── plumbing ─────────────────────────────────────────────────────────
    @property
    def connected(self) -> bool:
        return self._connected and self.modbus_index is not None

    def _call(self, client, request, what: str):
        """Call a bringup service and wait for the response (executor-safe).

        Uses call_async + bounded wait instead of Client.call() so it never
        spins the executor from inside one of its own callbacks.
        """
        if not client.service_is_ready() and not client.wait_for_service(
            timeout_sec=min(self.timeout, 1.0)
        ):
            raise TransportError(
                f"{what}: service {client.srv_name} is unavailable — "
                "dobot_bringup_v4_node is not running (transport:=bringup "
                "requires the arm driver)"
            )
        self.last_tx = what
        future = client.call_async(request)
        deadline = time.monotonic() + self.timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done():
            future.cancel()
            raise TransportError(
                f"{what}: no response from {client.srv_name} within "
                f"{self.timeout:.1f}s"
            )
        try:
            response = future.result()
        except Exception as exc:
            raise TransportError(
                f"{what}: {client.srv_name} call failed: {exc}"
            ) from exc
        if response is None:
            raise TransportError(f"{what}: empty response from {client.srv_name}")
        self.last_rx = f"res={response.res}"
        return response

    # ── low-level register access (index-explicit, used by probing) ──────
    def _read_via(self, index: int, address: int, count: int) -> list[int]:
        command = f"GetHoldRegs({index},{address},{count},U16)"
        request = GetHoldRegs.Request()
        request.index = index
        request.addr = address
        request.count = count
        request.val_type = "U16"
        reply = self._call(self._srv_get, request, command)
        if reply.res != 0:
            raise TransportError(f"Dobot rejected {command}: ErrorID={reply.res}")
        self.last_rx = reply.robot_return
        return _parse_u16_values(reply.robot_return, count, command)

    def _release_all_indices(self) -> None:
        """Best-effort ModbusClose(0..4) after a failed create+probe, so
        retries can never exhaust the controller's 5-master cap."""
        for index in _PROBE_INDICES:
            try:
                request = ModbusClose.Request()
                request.index = index
                if self._srv_close.service_is_ready():
                    self._call(self._srv_close, request, f"ModbusClose({index})")
            except Exception:
                pass

    # ── transport interface ──────────────────────────────────────────────
    def connect(self) -> None:
        with self._lock:
            self.close()
            try:
                request = SetTool485.Request()
                request.baudrate = self.baudrate
                request.parity = self.parity  # bringup's parser sends only the
                request.stop = self.stopbits  # baudrate (vendor: 8N1 default)
                request.identify = 1
                command = f"SetTool485({self.baudrate})"
                reply = self._call(self._srv_485, request, command)
                if reply.res != 0:
                    raise TransportError(
                        f"{command} rejected: ErrorID={reply.res}"
                    )

                # 1) adopt an existing readable master (e.g. left behind by a
                #    crashed previous run) instead of leaking a new one.
                errors = []
                for index in _PROBE_INDICES:
                    try:
                        self._read_via(index, 0x0200, 1)
                        self.modbus_index = index
                        break
                    except TransportError as exc:
                        errors.append(f"{index}: {exc}")

                # 2) none exists -> create our own gateway and find its index.
                if self.modbus_index is None:
                    request = ModbusCreate.Request()
                    # Embedded quotes: bringup streams the field raw, so this
                    # reproduces the vendor command byte-for-byte.
                    request.ip = '"127.0.0.1"'
                    request.port = 60000
                    request.slave_id = self.slave_id
                    request.is_rtu = 1  # RTU master on tool-RS485 port 60000
                    command = (
                        f'ModbusCreate("127.0.0.1",60000,{self.slave_id},1)'
                    )
                    reply = self._call(self._srv_create, request, command)
                    if reply.res != 0:
                        raise TransportError(
                            f"{command} rejected: ErrorID={reply.res} "
                            "(controller refused the RTU gateway — master cap "
                            "5; check robot state and free masters via "
                            "ModbusClose or controller restart)"
                        )
                    errors = []
                    for index in _PROBE_INDICES:
                        try:
                            self._read_via(index, 0x0200, 1)
                            self.modbus_index = index
                            break
                        except TransportError as exc:
                            errors.append(f"{index}: {exc}")
                    if self.modbus_index is None:
                        # Created but nothing answers: release everything so
                        # the retry loop cannot exhaust the master cap.
                        self._release_all_indices()
                        raise TransportError(
                            "RTU gateway created but register 0x0200 is "
                            "unreachable at every master index ("
                            + "; ".join(errors)
                            + ") — is the PGEA powered and on the tool RS485?"
                        )

                self._connected = True
            except Exception:
                self.close()
                raise

    def close(self) -> None:
        with self._lock:
            if self.modbus_index is not None:
                try:
                    request = ModbusClose.Request()
                    request.index = self.modbus_index
                    if self._srv_close.service_is_ready():
                        self._call(
                            self._srv_close,
                            request,
                            f"ModbusClose({self.modbus_index})",
                        )
                except Exception:
                    pass
            self.modbus_index = None
            self._connected = False

    def read_registers(self, address: int, count: int = 1) -> list[int]:
        if not 0 <= address <= 0xFFFF or not 1 <= count <= 4:
            raise ValueError(
                "Dobot gateway reads support address 0-65535 and count 1-4"
            )
        with self._lock:
            # Re-check + capture index atomically: close() may run concurrently
            # (link-loss / shutdown) and would otherwise hand None to the
            # generated int32 field setter.
            if not self.connected:
                raise TransportError(
                    "PGEA gateway via dobot_bringup is not connected"
                )
            return self._read_via(self.modbus_index, address, count)

    def read_register(self, address: int) -> int:
        return self.read_registers(address, 1)[0]

    def write_register(self, address: int, value: int) -> int:
        if not 0 <= address <= 0xFFFF or not 0 <= value <= 0xFFFF:
            raise ValueError("Register address and value must be unsigned 16-bit integers")
        command_addr, command_value = address, value
        with self._lock:
            if not self.connected:
                raise TransportError(
                    "PGEA gateway via dobot_bringup is not connected"
                )
            command = (
                f"SetHoldRegs({self.modbus_index},{command_addr},1,"
                f"{{{command_value}}},U16)"
            )
            request = SetHoldRegs.Request()
            request.index = self.modbus_index
            request.addr = command_addr
            request.count = 1
            request.val_tab = f"{{{command_value}}}"  # braces kept: parser streams raw
            request.val_type = "U16"
            reply = self._call(self._srv_set, request, command)
        if reply.res != 0:
            raise TransportError(f"Dobot rejected {command}: ErrorID={reply.res}")
        return value
