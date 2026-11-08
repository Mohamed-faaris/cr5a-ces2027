"""Transport backends for PGEA Modbus registers.

Dobot tool-flange mode sends CR Dashboard API commands to configure the tool
RS485 port and use the controller's internal RTU gateway. Direct serial mode is
for a PC USB-RS485 adapter wired to the gripper's RS485 bus.
"""
from __future__ import annotations
from dataclasses import dataclass
import re
import socket
import threading
import time
from typing import Optional

try:
    import serial
except ImportError:  # permit syntax/tests without pyserial installed
    serial = None

class TransportError(RuntimeError):
    pass

@dataclass
class DobotConfig:
    host: str = "192.168.5.1"
    dashboard_port: int = 29999
    slave_id: int = 1
    baudrate: int = 115200
    parity: str = "N"
    stopbits: int = 1
    timeout: float = 2.5

@dataclass
class SerialConfig:
    port: str = ""
    slave_id: int = 1
    baudrate: int = 115200
    parity: str = "N"
    stopbits: int = 1
    timeout: float = 0.5

@dataclass
class DobotReply:
    error_id: int
    values: list[str]
    raw: str

    @property
    def ok(self) -> bool:
        return self.error_id == 0

_REPLY_RE = re.compile(r"^\s*(-?\d+)\s*,\s*\{(.*?)\}\s*,", re.DOTALL)

def parse_dobot_reply(raw: str) -> DobotReply:
    """Parse CR Dashboard response: ErrorID,{value,...},command(...);"""
    match = _REPLY_RE.match(raw.strip())
    if not match:
        raise TransportError(f"Unrecognized Dobot Dashboard reply: {raw!r}")
    error_id = int(match.group(1))
    content = match.group(2).strip()
    values = [] if not content else [item.strip() for item in content.split(",")]
    return DobotReply(error_id, values, raw.strip())

class DobotDashboardClient:
    """Serialized client for Dobot CR Dashboard TCP service (normally port 29999)."""
    def __init__(self, host: str, port: int = 29999, timeout: float = 2.5):
        self.host, self.port, self.timeout = host, port, timeout
        self.sock: Optional[socket.socket] = None
        self.lock = threading.RLock()
        self.last_tx = ""
        self.last_rx = ""

    @property
    def connected(self) -> bool:
        return self.sock is not None

    def connect(self):
        self.close()
        try:
            s = socket.create_connection((self.host, self.port), timeout=self.timeout)
            s.settimeout(self.timeout)
            self.sock = s
        except OSError as exc:
            self.sock = None
            raise TransportError(
                f"Cannot connect to Dobot Dashboard at {self.host}:{self.port}: {exc}. "
                "Check controller LAN IP, PC subnet, cable and firmware."
            ) from exc

    def close(self):
        if self.sock is not None:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def command(self, command: str) -> DobotReply:
        if not self.sock:
            raise TransportError("Dobot Dashboard is not connected.")
        if not command or "\n" in command or "\r" in command or ";" in command:
            raise ValueError("Dashboard command must be a single command without terminator.")
        with self.lock:
            self.last_tx = command
            try:
                # CR dashboard commands are sent as ASCII ending in ')'; responses
                # are ASCII and end with ';'. Do not append an extra semicolon.
                self.sock.sendall(command.encode("ascii"))
                chunks = bytearray()
                deadline = time.monotonic() + self.timeout
                while b";" not in chunks:
                    if time.monotonic() >= deadline:
                        raise socket.timeout("timed out waiting for ';' response terminator")
                    part = self.sock.recv(4096)
                    if not part:
                        raise ConnectionError("Dobot closed the Dashboard socket")
                    chunks.extend(part)
                    if len(chunks) > 65536:
                        raise TransportError("Dashboard response exceeded 64 KiB")
                raw = bytes(chunks).split(b";", 1)[0].decode("ascii", errors="replace") + ";"
                self.last_rx = raw
            except (OSError, UnicodeError) as exc:
                raise TransportError(f"Dobot Dashboard command failed: {exc}") from exc
        reply = parse_dobot_reply(raw)
        if not reply.ok:
            raise TransportError(f"Dobot rejected {command!r}: ErrorID={reply.error_id}, reply={reply.raw}")
        return reply

class DobotToolRS485Transport:
    """PGEA Modbus through CR5A tool-flange RS485 via Dobot's local RTU gateway.

    Requires: PGEA connected to the tool connector's RS485 pins; CR controller
    firmware/API supporting SetTool485 and ModbusCreate RTU gateway (documented
    as controller v3.5.2+ for ModbusCreate/holding-register API).
    """
    def __init__(self, config: DobotConfig):
        if not 1 <= config.slave_id <= 247:
            raise ValueError("PGEA Modbus slave ID must be 1–247")
        if config.baudrate not in (4800, 9600, 19200, 38400, 57600, 115200):
            raise ValueError("Unsupported PGEA baud rate")
        if config.parity not in ("N", "E", "O"):
            raise ValueError("Parity must be N, E or O")
        if config.stopbits not in (1, 2):
            raise ValueError("Stop bits must be 1 or 2")
        self.config = config
        self.dashboard = DobotDashboardClient(config.host, config.dashboard_port, config.timeout)
        self.modbus_index: Optional[int] = None
        self.last_tx = ""
        self.last_rx = ""

    @property
    def connected(self) -> bool:
        return self.dashboard.connected and self.modbus_index is not None

    def connect(self):
        self.dashboard.connect()
        try:
            # Read-only query confirms this is a live CR Dashboard endpoint.
            self.dashboard.command("RobotMode()")
            self.dashboard.command(f'SetTool485({self.config.baudrate},"{self.config.parity}",{self.config.stopbits})')
            # In the CR controller API this special combination routes Modbus RTU
            # to the robot's end-tool RS485 connector through internal port 60000.
            reply = self.dashboard.command(
                f'ModbusCreate("127.0.0.1",60000,{self.config.slave_id},1)'
            )
            if not reply.values or not re.fullmatch(r"-?\d+", reply.values[0]):
                raise TransportError(f"ModbusCreate returned no master index: {reply.raw}")
            self.modbus_index = int(reply.values[0])
            if self.modbus_index < 0:
                raise TransportError(f"Dobot returned invalid Modbus master index: {self.modbus_index}")
            # Read a harmless feedback register to verify end-to-end reachability.
            self.read_register(0x0200)
        except Exception:
            self.close()
            raise

    def close(self):
        if self.modbus_index is not None and self.dashboard.connected:
            try:
                self.dashboard.command(f"ModbusClose({self.modbus_index})")
            except Exception:
                pass
        self.modbus_index = None
        self.dashboard.close()

    def read_registers(self, address: int, count: int = 1) -> list[int]:
        if not self.connected:
            raise TransportError("Dobot tool-RS485 gateway is not connected")
        if not 0 <= address <= 0xFFFF or not 1 <= count <= 4:
            raise ValueError("Dobot gateway reads support address 0–65535 and count 1–4")
        command = f"GetHoldRegs({self.modbus_index},{address},{count},U16)"
        self.last_tx = command
        reply = self.dashboard.command(command)
        self.last_rx = reply.raw
        if len(reply.values) != count:
            raise TransportError(f"Expected {count} register value(s), got reply {reply.raw!r}")
        try:
            values = [int(v, 0) for v in reply.values]
        except ValueError as exc:
            raise TransportError(f"Non-integer register reply: {reply.raw!r}") from exc
        if any(not 0 <= v <= 0xFFFF for v in values):
            raise TransportError(f"Register value outside unsigned 16-bit range: {values}")
        return values

    def read_register(self, address: int) -> int:
        return self.read_registers(address, 1)[0]

    def write_register(self, address: int, value: int) -> int:
        if not self.connected:
            raise TransportError("Dobot tool-RS485 gateway is not connected")
        if not 0 <= address <= 0xFFFF or not 0 <= value <= 0xFFFF:
            raise ValueError("Register address and value must be unsigned 16-bit integers")
        command = f"SetHoldRegs({self.modbus_index},{address},1,{{{value}}},U16)"
        self.last_tx = command
        reply = self.dashboard.command(command)
        self.last_rx = reply.raw
        return value

class DirectSerialTransport:
    """Direct USB-RS485 transport. Includes Modbus RTU framing/CRC checks."""
    def __init__(self, config: SerialConfig):
        if not config.port:
            raise ValueError("Select a PC serial port")
        if not 1 <= config.slave_id <= 247:
            raise ValueError("PGEA Modbus slave ID must be 1–247")
        self.config = config
        self.ser = None
        self.lock = threading.RLock()
        self.last_tx = ""
        self.last_rx = ""

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    @staticmethod
    def crc16(data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        return crc & 0xFFFF

    @classmethod
    def frame(cls, payload: bytes) -> bytes:
        crc = cls.crc16(payload)
        return payload + bytes((crc & 0xFF, (crc >> 8) & 0xFF))

    @classmethod
    def valid_crc(cls, frame: bytes) -> bool:
        if len(frame) < 4:
            return False
        crc = cls.crc16(frame[:-2])
        return frame[-2:] == bytes((crc & 0xFF, (crc >> 8) & 0xFF))

    def connect(self):
        self.close()
        if serial is None:
            raise TransportError("pyserial is not installed; run pip install -r requirements.txt")
        try:
            self.ser = serial.Serial(
                port=self.config.port, baudrate=self.config.baudrate,
                bytesize=8, parity=self.config.parity, stopbits=self.config.stopbits,
                timeout=self.config.timeout, write_timeout=self.config.timeout,
                rtscts=False, dsrdtr=False
            )
            time.sleep(0.05)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            # End-to-end test using a read-only register.
            self.read_register(0x0200)
        except Exception as exc:
            self.close()
            if isinstance(exc, (ValueError, TransportError)):
                raise
            raise TransportError(f"Could not open/test serial port: {exc}") from exc

    def close(self):
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            finally:
                self.ser = None

    def _read_exact(self, count: int) -> bytes:
        output = bytearray()
        while len(output) < count:
            chunk = self.ser.read(count - len(output))
            if not chunk:
                raise TransportError(f"RTU timeout: expected {count} more byte(s), received {len(output)}")
            output.extend(chunk)
        return bytes(output)

    def _request(self, payload: bytes, function: int) -> bytes:
        if not self.connected:
            raise TransportError("PC serial port is not connected")
        request = self.frame(payload)
        with self.lock:
            try:
                self.ser.reset_input_buffer()
                self.last_tx = request.hex(" ").upper()
                self.ser.write(request)
                self.ser.flush()
                header = self._read_exact(2)
                response_func = header[1]
                if response_func == (function | 0x80):
                    remainder = self._read_exact(3)  # exception code + CRC
                    response = header + remainder
                elif response_func != function:
                    # Read a bounded common response size to report the mismatch
                    # rather than allowing a partial response to leak to the next request.
                    remainder = self._read_exact(6 if function == 0x06 else 3)
                    response = header + remainder
                elif function == 0x03:
                    byte_count = self._read_exact(1)
                    response = header + byte_count + self._read_exact(byte_count[0] + 2)
                elif function == 0x06:
                    response = header + self._read_exact(6)
                else:
                    raise TransportError(f"Unsupported function code 0x{function:02X}")
                self.last_rx = response.hex(" ").upper()
            except (OSError, TimeoutError) as exc:
                raise TransportError(f"Serial I/O error: {exc}") from exc
        if not self.valid_crc(response):
            raise TransportError(f"Invalid RTU CRC. RX={self.last_rx}")
        if response[0] != self.config.slave_id:
            raise TransportError(f"Unexpected RTU slave ID {response[0]}; expected {self.config.slave_id}")
        if response[1] & 0x80:
            if len(response) >= 5:
                raise TransportError(f"Modbus exception response, function=0x{response[1]:02X}, code=0x{response[2]:02X}")
            raise TransportError("Malformed Modbus exception response")
        if response[1] != function:
            raise TransportError(f"Unexpected RTU function 0x{response[1]:02X}")
        return response

    def read_registers(self, address: int, count: int = 1) -> list[int]:
        if not 0 <= address <= 0xFFFF or not 1 <= count <= 125:
            raise ValueError("Invalid read address/count")
        payload = bytes((self.config.slave_id, 0x03, address >> 8, address & 255, count >> 8, count & 255))
        response = self._request(payload, 0x03)
        if response[2] != count * 2:
            raise TransportError(f"Wrong RTU byte count {response[2]}; expected {count*2}")
        return [int.from_bytes(response[3+i*2:5+i*2], "big") for i in range(count)]

    def read_register(self, address: int) -> int:
        return self.read_registers(address, 1)[0]

    def write_register(self, address: int, value: int) -> int:
        if not 0 <= address <= 0xFFFF or not 0 <= value <= 0xFFFF:
            raise ValueError("Register address/value must be unsigned 16-bit integers")
        payload = bytes((self.config.slave_id, 0x06, address >> 8, address & 255, value >> 8, value & 255))
        response = self._request(payload, 0x06)
        if response[:6] != payload:
            raise TransportError(f"Write acknowledgement mismatch. RX={self.last_rx}")
        return value
