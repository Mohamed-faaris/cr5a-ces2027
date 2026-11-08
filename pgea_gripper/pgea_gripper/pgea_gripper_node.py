"""ROS 2 bridge for the DH-Robotics PGEA gripper on a real DOBOT CR5A.

Wraps the proven Modbus layer of the standalone `pgea_dobot_controller`
project (pgea_app: PGEADriver + transports, vendored byte-identical under
pgea_gripper/pgea_app/) into a control_msgs/GripperCommand action server at
/gripper_action_controller/gripper_cmd — the exact action the cr5_pgea MoveIt
controller config (cr5_pgea.moveit_controllers.yaml) routes gripper goals to —
so MoveIt plan-and-execute works against the physical gripper unchanged.

Coordinate mapping (matches the cr5_pgea URDF and the PGEA V4.0 manual):
    0.000 m  <-> 0 per mille   (fingers fully open)
    0.020 m  <-> 1000 per mille (fully closed, per-finger drive joint)

Transports:
    bringup (default): PGEA wired to the robot tool-flange RS485 pins; all
        gateway commands (SetTool485/ModbusCreate/GetHoldRegs/SetHoldRegs)
        go through dobot_bringup_v4's ROS services on the arm driver's own
        Dashboard connection. REQUIRED in the full real-robot stack: the CR
        controller serves a single Dashboard TCP client (:29999), owned by
        dobot_bringup_v4_node — a second client connects but its commands are
        dropped ("Dobot closed the Dashboard socket" = "opening, no
        response"). Needs dobot_bringup_v4_node running (bringup_transport.py).
    dashboard: standalone Dashboard TCP client — only when the arm driver is
        NOT running (bench-testing the gripper alone).
    serial: PC USB-RS485 adapter on the PGEA bus (raw Modbus RTU, pyserial).

The node also publishes sensor_msgs/JointState for left_finger_joint and its
mimic right_finger_joint on /joint_states from feedback register 0x0202, so
move_group/RSP know the finger state on the real robot (the arm driver only
publishes joint1..joint6).
"""
from __future__ import annotations

import json
import math
import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from pgea_gripper_interfaces.action import GripUntilForce
from pgea_gripper_interfaces.srv import ClearAlarm, GetRegister, GetStatus, Jog, SetIoGroup, SetRegister

from pgea_gripper.bringup_transport import BringupServiceTransport
from pgea_gripper.pgea_app.driver import PGEADriver
from pgea_gripper.pgea_app.register_map import ALARMS, GRIP_STATES, INIT_STATES, REGISTERS, WRITABLE_REGS
from pgea_gripper.pgea_app.transport import (
    DobotConfig,
    DirectSerialTransport,
    DobotToolRS485Transport,
    SerialConfig,
    TransportError,
)

# Per-finger drive stroke: URDF 0..0.020 m <-> manual 0..1000 per mille.
STROKE_M = 0.020

GRIP_MOVING = 0
GRIP_IN_POSITION = 1
GRIP_OBJECT_CAUGHT = 2
GRIP_OBJECT_DROPPED = 3  # transient while opening; keep polling until in position.


def m_to_permille(position_m: float) -> int:
    """Convert per-finger metres to the PGEA 0..1000 command range."""
    if not math.isfinite(position_m) or not 0.0 <= position_m <= STROKE_M:
        raise ValueError(
            f"Position must be finite and between 0 and {STROKE_M:.3f} m per finger"
        )
    return int(round(position_m / STROKE_M * 1000))


def permille_to_m(position_pm: int) -> float:
    return position_pm / 1000.0 * STROKE_M


def resolve_robot_ip(configured: str) -> tuple[str, str]:
    """robot_ip param -> $IP_address -> dobot_bringup param.json -> 192.168.5.1."""
    if configured:
        return configured, "parameter"
    env_ip = os.getenv("IP_address", "")
    if env_ip:
        return env_ip, "environment IP_address"
    try:
        path = os.path.join(
            get_package_share_directory("dobot_bringup_v4"), "config", "param.json"
        )
        with open(path, "r") as handle:
            data = json.load(handle)
        index = data.get("current_robot", 1) - 1
        return data["node_info"][index]["ip_address"], "dobot_bringup param.json"
    except Exception:
        return "192.168.5.1", "default"


class PGEAGripperNode(Node):
    def __init__(self) -> None:
        super().__init__("pgea_gripper")

        # ── parameters ────────────────────────────────────────────────
        self.declare_parameter("transport", "bringup")         # bringup | dashboard | serial
        self.declare_parameter("robot_ip", "")                  # '' = auto-resolve
        self.declare_parameter("dashboard_port", 29999)
        self.declare_parameter("slave_id", 1)
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("parity", "N")
        self.declare_parameter("stopbits", 1)
        self.declare_parameter("io_timeout", 2.5)
        self.declare_parameter("serial_port", "")               # required for serial
        self.declare_parameter("force_percent", 60)             # 20..100
        self.declare_parameter("speed_percent", 50)             # 1..100
        self.declare_parameter("auto_initialize", False)        # motion may calibrate
        self.declare_parameter("init_timeout", 60.0)
        self.declare_parameter("move_timeout", 15.0)
        self.declare_parameter("goal_tolerance", 0.001)         # metres
        self.declare_parameter("feedback_rate", 10.0)           # Hz
        self.declare_parameter("reconnect_period", 5.0)         # s
        self.declare_parameter("jog_timeout", 3.0)              # s; auto-stop manual jog

        self._mode = str(self.get_parameter("transport").value).lower()
        self._force_percent = int(self.get_parameter("force_percent").value)
        self._speed_percent = int(self.get_parameter("speed_percent").value)
        self._auto_initialize = bool(self.get_parameter("auto_initialize").value)
        self._init_timeout = float(self.get_parameter("init_timeout").value)
        self._move_timeout = float(self.get_parameter("move_timeout").value)
        self._goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self._reconnect_period = float(self.get_parameter("reconnect_period").value)
        self._jog_timeout = float(self.get_parameter("jog_timeout").value)
        if not math.isfinite(self._jog_timeout) or self._jog_timeout <= 0.0:
            raise ValueError("jog_timeout must be a finite positive duration")
        self._force_percent = max(20, min(100, self._force_percent))
        self._speed_percent = max(1, min(100, self._speed_percent))

        configured_ip = str(self.get_parameter("robot_ip").value)
        if self._mode in ("dashboard", "bringup"):
            self._host, host_source = resolve_robot_ip(configured_ip)
        else:
            self._host = configured_ip
            host_source = ""

        # ── state ─────────────────────────────────────────────────────
        self._driver: PGEADriver | None = None
        self._io_lock = threading.RLock()
        # The position and force-grip action servers share one physical motor.
        # Reject overlapping goals instead of allowing two callbacks to race
        # writes to the PGEA command registers.
        self._motion_state_lock = threading.Lock()
        self._active_motion: str | None = None
        self.add_on_set_parameters_callback(self._on_profile_parameters)
        self._jog_deadline: float | None = None
        self._last_position_m = 0.0
        self._last_init_state: int | None = None
        self._next_connect_attempt = 0.0
        self._read_failures = 0

        # ── action server (exact name MoveIt's cr5_pgea config expects) ──
        self._action_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            GripperCommand,
            "/gripper_action_controller/gripper_cmd",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_group,
        )

        # ── services ──────────────────────────────────────────────────
        self.create_service(Trigger, "~/initialize", self._on_initialize)
        self.create_service(Trigger, "~/calibrate", self._on_calibrate)

        self.create_service(GetRegister, "~/get_register", self._on_get_register)
        self.create_service(SetRegister, "~/set_register", self._on_set_register)
        self.create_service(GetStatus, "~/get_status", self._on_get_status)
        self.create_service(SetIoGroup, "~/set_io_group", self._on_set_io_group)
        self.create_service(Jog, "~/jog", self._on_jog)
        self.create_service(ClearAlarm, "~/clear_alarm", self._on_clear_alarm)
        self._force_action = ActionServer(
            self, GripUntilForce, "~/grip_until_force",
            execute_callback=self._on_grip_until_force,
            goal_callback=self._force_goal_callback,
            cancel_callback=self._force_cancel_callback,
            callback_group=self._action_group,
        )

        # ── feedback: finger joint states + reconnect handling ────────
        self._state_pub = self.create_publisher(JointState, "joint_states", 10)
        feedback_rate = max(1.0, float(self.get_parameter("feedback_rate").value))
        self.create_timer(1.0 / feedback_rate, self._feedback_tick)
        self.create_timer(0.1, self._jog_watchdog_tick)

        if self._mode == "bringup":
            detail = f"dobot_bringup_ros2 services (robot {self._host})"
        elif self._mode == "dashboard":
            detail = (
                f"{self._host} (Dashboard :{self.get_parameter('dashboard_port').value})"
            )
        else:
            detail = f"device '{self.get_parameter('serial_port').value or 'unset'}'"
        self.get_logger().info(
            f"PGEA bridge up: transport={self._mode}, target={detail}"
            + (f" [robot_ip from {host_source}]" if host_source else "")
            + ". Waiting for first connection..."
        )

    def _on_profile_parameters(self, parameters):
        """Apply force/speed parameter changes to the values used by motion."""
        updates = {}
        for parameter in parameters:
            if parameter.name not in ("force_percent", "speed_percent"):
                continue
            value = parameter.value
            if isinstance(value, bool) or not isinstance(value, int):
                return SetParametersResult(
                    successful=False,
                    reason=f"{parameter.name} must be an integer percentage",
                )
            minimum = 20 if parameter.name == "force_percent" else 1
            if not minimum <= value <= 100:
                return SetParametersResult(
                    successful=False,
                    reason=f"{parameter.name} must be between {minimum} and 100",
                )
            updates[parameter.name] = value

        if updates:
            with self._motion_state_lock:
                if self._active_motion is not None:
                    return SetParametersResult(
                        successful=False,
                        reason=f"Cannot change gripper profile while {self._active_motion} is active",
                    )
                if "force_percent" in updates:
                    self._force_percent = updates["force_percent"]
                if "speed_percent" in updates:
                    self._speed_percent = updates["speed_percent"]
        return SetParametersResult(successful=True)

    # ── connection management ─────────────────────────────────────────
    def _build_driver(self) -> PGEADriver:
        slave = int(self.get_parameter("slave_id").value)
        baud = int(self.get_parameter("baudrate").value)
        parity = str(self.get_parameter("parity").value)
        stopbits = int(self.get_parameter("stopbits").value)
        timeout = float(self.get_parameter("io_timeout").value)
        if self._mode == "dashboard":
            config = DobotConfig(
                host=self._host,
                dashboard_port=int(self.get_parameter("dashboard_port").value),
                slave_id=slave,
                baudrate=baud,
                parity=parity,
                stopbits=stopbits,
                timeout=timeout,
            )
            transport = DobotToolRS485Transport(config)
        elif self._mode == "bringup":
            transport = BringupServiceTransport(
                self,
                slave_id=slave,
                baudrate=baud,
                parity=parity,
                stopbits=stopbits,
                timeout=timeout,
            )
        elif self._mode == "serial":
            port = str(self.get_parameter("serial_port").value)
            if not port:
                raise TransportError("transport:=serial requires the serial_port parameter")
            config = SerialConfig(
                port=port, slave_id=slave, baudrate=baud,
                parity=parity, stopbits=stopbits, timeout=timeout,
            )
            transport = DirectSerialTransport(config)
        else:
            raise TransportError(
                f"Unknown transport '{self._mode}' "
                "(expected 'bringup', 'dashboard' or 'serial')"
            )
        transport.connect()
        return PGEADriver(transport)

    def _ensure_connected(self) -> PGEADriver:
        with self._io_lock:
            if self._driver is not None and self._driver.connected:
                return self._driver
            driver = self._build_driver()
            self._driver = driver
            self._read_failures = 0
            where = (
                "dobot_bringup_ros2 Modbus services (tool-RS485 gateway)"
                if self._mode == "bringup"
                else f"{self._host}:29999 tool-RS485 gateway"
                if self._mode == "dashboard"
                else f"serial {self.get_parameter('serial_port').value}"
            )
            self.get_logger().info(f"PGEA connected via {where}")
            return driver

    def _maybe_reconnect(self) -> None:
        now = time.monotonic()
        if now < self._next_connect_attempt:
            return
        self._next_connect_attempt = now + self._reconnect_period
        try:
            self._ensure_connected()
        except (TransportError, ValueError) as exc:
            self.get_logger().error(
                f"PGEA not reachable: {exc}", throttle_duration_sec=self._reconnect_period
            )

    def _close_driver(self) -> None:
        with self._io_lock:
            if self._driver is not None:
                try:
                    self._driver.close()
                except Exception:
                    pass
                self._driver = None

    # ── register helpers ──────────────────────────────────────────────
    def _read_state(self, driver: PGEADriver) -> tuple[int, int, int, int]:
        """One snapshot: (init_state, grip_state, position_per_mille, alarm)."""
        init_state, grip, position = driver.read_registers(0x0200, 3)
        alarm = driver.read_register(0x0205)
        return init_state, grip, position, alarm

    def _safe_stop(self, driver: PGEADriver | None) -> None:
        if driver is None:
            return
        try:
            driver.jog(0)
        except Exception as exc:
            self.get_logger().warn(f"Stop command failed: {exc}")

    def _ensure_initialized(self, driver: PGEADriver) -> None:
        state = driver.read_register(0x0200)
        if state == 1:
            return
        if state == 0xFFFF:
            raise TransportError(
                "PGEA reports a calibration error (0x0200=0xFFFF). Clear the "
                "obstruction and recalibrate (~/calibrate) before commanding motion."
            )
        if state != 2 and not self._auto_initialize:
            raise TransportError(
                f"PGEA is not initialized (0x0200={INIT_STATES.get(state, state)}). "
                "Call ~/initialize (or ~/calibrate), or relaunch with "
                "auto_initialize:=true. Motion is blocked until initialization completes."
            )
        if state != 2:  # state 0: start directional initialization
            self.get_logger().warn(
                "PGEA not initialized — running directional initialization "
                "(fingers will move); use auto_initialize:=true to suppress this"
            )
            driver.initialize(full_calibration=False)
        deadline = time.monotonic() + self._init_timeout
        while time.monotonic() < deadline:
            state = driver.read_register(0x0200)
            if state == 1:
                self.get_logger().info("PGEA initialization complete")
                return
            if state == 0xFFFF:
                raise TransportError("PGEA calibration failed (0x0200=0xFFFF)")
            time.sleep(0.2)
        raise TransportError(f"PGEA initialization timed out after {self._init_timeout:.0f}s")

    # ── services ──────────────────────────────────────────────────────
    def _on_initialize(self, request, response):
        if not self._claim_motion("initialize"):
            response.success = False
            response.message = "PGEA is already executing another motion"
            return response
        try:
            return self._run_initialize(request, response)
        finally:
            self._release_motion("initialize")

    def _run_initialize(self, _request, response):
        try:
            driver = self._ensure_connected()
            if driver.read_register(0x0200) == 1:
                response.success = True
                response.message = "PGEA already initialized"
                return response
            driver.initialize(full_calibration=False)
            deadline = time.monotonic() + self._init_timeout
            while time.monotonic() < deadline:
                state = driver.read_register(0x0200)
                if state == 1:
                    response.success = True
                    response.message = "Initialization complete"
                    return response
                if state == 0xFFFF:
                    response.success = False
                    response.message = "Calibration error (0xFFFF); check obstructions"
                    return response
                time.sleep(0.2)
            response.success = False
            response.message = f"Initialization timed out after {self._init_timeout:.0f}s"
        except (TransportError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _on_calibrate(self, request, response):
        """Reserve the single PGEA motor before full calibration."""
        if not self._claim_motion("calibrate"):
            response.success = False
            response.message = "PGEA is already executing another motion"
            return response
        try:
            return self._run_calibrate(request, response)
        finally:
            self._release_motion("calibrate")

    def _run_calibrate(self, _request, response):
        """Full close-to-open calibration (0x0100=0xA5) — re-measures stroke."""
        try:
            driver = self._ensure_connected()
            driver.initialize(full_calibration=True)
            deadline = time.monotonic() + self._init_timeout
            while time.monotonic() < deadline:
                state = driver.read_register(0x0200)
                if state == 1:
                    response.success = True
                    response.message = "Full calibration complete"
                    return response
                if state == 0xFFFF:
                    response.success = False
                    response.message = "Calibration error (0xFFFF); clear obstruction and retry"
                    return response
                time.sleep(0.2)
            response.success = False
            response.message = f"Calibration timed out after {self._init_timeout:.0f}s"
        except (TransportError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
        return response

    # ── GripperCommand action ─────────────────────────────────────────

    # Typed PGEA diagnostics and register-level controls.
    def _on_get_register(self, request, response):
        try:
            address = int(request.address)
            if address not in set(REGISTERS.values()):
                raise ValueError("Only documented PGEA registers may be read")
            with self._io_lock:
                driver = self._ensure_connected()
                response.value = int(driver.read_register(address))
            response.success = True
            response.register_name = next((n for n, a in REGISTERS.items() if a == address), hex(address))
            response.message = "read ok"
        except Exception as exc:
            response.success = False
            response.value = 0
            response.register_name = ""
            response.message = str(exc)
        return response

    def _on_set_register(self, request, response):
        try:
            address, value = int(request.address), int(request.value)
            if address in (0x0100, 0x0103, 0x010A):
                raise ValueError(
                    "Initialization, position and jog commands must use their dedicated ROS action/service interfaces"
                )
            if address not in WRITABLE_REGS:
                raise ValueError("Write blocked: register is not in the documented writable allowlist")
            label, low, high = WRITABLE_REGS[address]
            if not low <= value <= high:
                raise ValueError(f"{label} must be between {low} and {high}")
            if not self._claim_motion("register-write"):
                raise ValueError("PGEA is executing another motion")
            try:
                with self._io_lock:
                    self._ensure_connected().safe_write_register(address, value)
            finally:
                self._release_motion("register-write")
            response.success, response.value = True, value
            response.message = f"wrote {label}"
        except Exception as exc:
            response.success, response.value = False, 0
            response.message = str(exc)
        return response

    def _on_get_status(self, _request, response):
        try:
            with self._io_lock:
                snapshot = self._ensure_connected().read_snapshot()
            alarm = int(snapshot.get("error") or 0)
            snapshot["init_text"] = INIT_STATES.get(snapshot.get("init"), "unknown")
            snapshot["grip_text"] = GRIP_STATES.get(snapshot.get("grip"), "unknown")
            snapshot["alarm"] = {"code": alarm,
                "name": ALARMS.get(alarm, (f"Unknown alarm 0x{alarm:02X}", ""))[0],
                "remedy": ALARMS.get(alarm, ("", "Consult the PGEA manual."))[1]}
            response.success, response.status_json = True, json.dumps(snapshot)
            response.message = "status read ok"
        except Exception as exc:
            response.success, response.status_json = False, "{}"
            response.message = str(exc)
        return response

    def _on_set_io_group(self, request, response):
        try:
            group, pos = int(request.group), int(request.position)
            force, speed = int(request.force_percent), int(request.speed_percent)
            if group not in (1, 2, 3, 4): raise ValueError("I/O group must be 1..4")
            if not 0 <= pos <= 1000: raise ValueError("Position must be 0..1000 per mille")
            if not 20 <= force <= 100 or not 1 <= speed <= 100:
                raise ValueError("Force must be 20..100%; speed 1..100%")
            if not self._claim_motion("io-preset-write"):
                raise ValueError("PGEA is executing another motion")
            try:
                with self._io_lock:
                    driver = self._ensure_connected()
                    driver._require_initialized()
                    base = 0x0405 + (group - 1) * 3
                    driver.write_register(base, pos)
                    driver.write_register(base + 1, force)
                    driver.write_register(base + 2, speed)
            finally:
                self._release_motion("io-preset-write")
            response.success, response.message = True, f"saved I/O group {group} preset"
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def _on_jog(self, request, response):
        claimed = False
        direction = None
        try:
            direction = int(request.direction)
            if direction not in (-1, 0, 1):
                raise ValueError("Direction must be -1, 0, or 1")
            if direction:
                if not self._claim_motion("jog"):
                    raise ValueError("PGEA is already executing another motion")
                claimed = True
            with self._io_lock:
                self._ensure_connected().jog(direction)
            with self._motion_state_lock:
                self._jog_deadline = (
                    time.monotonic() + self._jog_timeout if direction else None
                )
                if direction == 0 and self._active_motion in (
                        "jog", "jog-stop-failed"):
                    self._active_motion = None
            response.success = True
            response.message = {
                -1: f"reverse jog commanded; auto-stop in {self._jog_timeout:.1f}s or send direction 0",
                0: "stop commanded",
                1: f"forward jog commanded; auto-stop in {self._jog_timeout:.1f}s or send direction 0",
            }[direction]
        except Exception as exc:
            with self._motion_state_lock:
                self._jog_deadline = None
                if direction == 0 and self._active_motion in (
                        "jog", "jog-stop-failed", "jog-stopping"):
                    # A failed stop must not unlock the motor or allow a new
                    # motion. Keep it reserved until an explicit stop succeeds.
                    self._active_motion = "jog-stop-failed"
            if claimed:
                self._release_motion("jog")
            response.success, response.message = False, str(exc)
        return response

    def _jog_watchdog_tick(self) -> None:
        """Stop a manual jog if the operator does not issue an explicit stop."""
        now = time.monotonic()
        with self._motion_state_lock:
            if (self._active_motion != "jog" or self._jog_deadline is None
                    or now < self._jog_deadline):
                return
            # Reserve a stopping state so no new command can race this stop.
            self._active_motion = "jog-stopping"
            self._jog_deadline = None
        stopped = False
        try:
            with self._io_lock:
                if self._driver is None or not self._driver.connected:
                    raise TransportError(
                        "PGEA transport unavailable; automatic jog stop could not be sent")
                self._driver.jog(0)
            stopped = True
            self.get_logger().warn(
                f"PGEA jog exceeded {self._jog_timeout:.1f}s; automatic stop sent")
        except Exception as exc:
            self.get_logger().error(
                f"PGEA jog watchdog stop failed: {exc}; gripper stays motion-locked")
        finally:
            with self._motion_state_lock:
                self._active_motion = None if stopped else "jog-stop-failed"

    def _on_clear_alarm(self, _request, response):
        if not self._claim_motion("clear-alarm"):
            response.success = False
            response.message = "Stop the active gripper operation before clearing its alarm"
            return response
        try:
            with self._io_lock:
                self._ensure_connected().safe_write_register(0x0503, 1)
            response.success, response.message = True, "clear-alarm command sent; verify feedback"
        except Exception as exc:
            response.success, response.message = False, str(exc)
        finally:
            self._release_motion("clear-alarm")
        return response

    def _claim_motion(self, operation: str) -> bool:
        with self._motion_state_lock:
            if self._active_motion is not None:
                self.get_logger().warn(
                    f"Rejecting {operation}: {self._active_motion} is already active"
                )
                return False
            self._active_motion = operation
            return True

    def _release_motion(self, operation: str) -> None:
        with self._motion_state_lock:
            if self._active_motion == operation:
                self._active_motion = None

    def _force_goal_callback(self, _goal_request):
        return (GoalResponse.ACCEPT if self._claim_motion("force-grip")
                else GoalResponse.REJECT)

    @staticmethod
    def _force_cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def _on_grip_until_force(self, goal_handle):
        try:
            return self._execute_force_grip(goal_handle)
        finally:
            self._release_motion("force-grip")

    def _execute_force_grip(self, goal_handle):
        goal, result = goal_handle.request, GripUntilForce.Result()
        try:
            force, speed = int(goal.force_percent), int(goal.speed_percent)
            close_pos = int(goal.close_position)
            timeout_ms = int(goal.timeout_ms)
            if not 100 <= timeout_ms <= 120000:
                raise ValueError("timeout_ms must be between 100 and 120000")
            timeout = timeout_ms / 1000.0
            max_current = int(goal.max_current) if goal.use_current_limit else None
            def canceled():
                return bool(goal_handle.is_cancel_requested)
            def progress(snap):
                fb = GripUntilForce.Feedback()
                fb.outcome = str(snap.get("outcome", "closing"))
                fb.position = int(snap.get("position") or 0)
                fb.grip_status = int(snap.get("grip") or 0)
                fb.current = int(snap.get("current") or 0)
                fb.alarm = int(snap.get("alarm") or 0)
                fb.elapsed = float(snap.get("elapsed") or 0.0)
                goal_handle.publish_feedback(fb)
            with self._io_lock:
                snap = self._ensure_connected().close_until_force(
                    force_limit=force, speed=speed, close_position=close_pos,
                    timeout=timeout, max_current=max_current,
                    progress=progress, cancel=canceled)
            result.success = snap.get("outcome") == "object_caught"
            result.outcome = str(snap.get("outcome", "unknown"))
            result.position = int(snap.get("position") or 0)
            result.grip_status = int(snap.get("grip") or 0)
            result.current = int(snap.get("current") or 0)
            result.alarm = int(snap.get("alarm") or 0)
            result.elapsed = float(snap.get("elapsed") or 0.0)
            result.message = str(snap.get("error") or result.outcome)
            if result.success: goal_handle.succeed()
            elif result.outcome == "canceled": goal_handle.canceled()
            else: goal_handle.abort()
        except Exception as exc:
            result.success, result.outcome = False, "error"
            result.position = result.grip_status = result.current = result.alarm = 0
            result.elapsed, result.message = 0.0, str(exc)
            goal_handle.abort()
        return result

    def _goal_callback(self, _goal_request) -> GoalResponse:
        return (GoalResponse.ACCEPT if self._claim_motion("position")
                else GoalResponse.REJECT)

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        try:
            return self._execute_position(goal_handle)
        finally:
            self._release_motion("position")

    def _execute_position(self, goal_handle):
        request = goal_handle.request
        result = GripperCommand.Result()
        result.position = self._last_position_m
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = False

        try:
            target_m = float(request.command.position)
            target_pm = m_to_permille(target_m)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"Invalid GripperCommand position: {exc}")
            goal_handle.abort()
            return result
        # Humble's GripperCommand goal carries no tolerance field; the param
        # (0.001 m, matching the cr5_pgea joint_limits goal_tolerance) applies.
        tolerance = max(self._goal_tolerance, 1e-6)
        # control_msgs expresses max_effort in Newtons while this PGEA
        # register accepts a percentage. There is no calibrated N-to-% curve
        # in this integration, so positive values must not be reinterpreted.
        try:
            max_effort = float(request.command.max_effort)
            if not math.isfinite(max_effort) or max_effort != 0.0:
                raise ValueError(
                    "positive max_effort requires a calibrated PGEA N-to-% mapping; "
                    "use max_effort=0 and configure force_percent"
                )
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"Unsupported GripperCommand effort: {exc}")
            goal_handle.abort()
            return result
        force = self._force_percent
        try:
            driver = self._ensure_connected()
            self._ensure_initialized(driver)
            driver.move(target_pm, force, self._speed_percent)
        except (TransportError, ValueError) as exc:
            self.get_logger().error(f"GripperCommand({target_m:.3f} m) failed: {exc}")
            goal_handle.abort()
            return result

        self.get_logger().info(
            f"Gripper moving to {target_m:.3f} m ({target_pm}‰), "
            f"force={force}% speed={self._speed_percent}%"
        )

        deadline = time.monotonic() + self._move_timeout
        actual_m = self._last_position_m
        stalled = False
        stopped = False
        saw_motion = False
        alarm = 0

        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self._safe_stop(driver)
                goal_handle.canceled()
                result.position = actual_m
                self.get_logger().info("Gripper goal canceled — stop commanded")
                return result
            try:
                _init_state, grip, position_pm, alarm = self._read_state(driver)
            except TransportError as exc:
                self.get_logger().warn(
                    f"Feedback read failed while executing: {exc}",
                    throttle_duration_sec=2.0,
                )
                time.sleep(0.1)
                continue
            actual_m = permille_to_m(position_pm)
            self._last_position_m = actual_m
            feedback = GripperCommand.Feedback()
            feedback.position = actual_m
            feedback.effort = 0.0
            goal_handle.publish_feedback(feedback)
            if alarm:
                self.get_logger().error(
                    f"PGEA alarm during motion: "
                    f"{ALARMS.get(alarm, (f'code 0x{alarm:02X}', ''))[0]}"
                )
                self._safe_stop(driver)
                break
            if grip in (GRIP_MOVING, GRIP_OBJECT_DROPPED):
                saw_motion = True
            if grip == GRIP_IN_POSITION:
                stopped = True
                break
            if grip == GRIP_OBJECT_CAUGHT:
                # An object-caught state confirms a grasp while closing, but
                # must never satisfy an OPEN command. During opening, ignore
                # a stale caught state until the feedback shows motion/drop;
                # a persistent caught state will time out and abort safely.
                if target_m <= 1e-6:
                    if saw_motion:
                        stopped = True
                        stalled = True
                        break
                    time.sleep(0.05)
                    continue
                stopped = True
                stalled = True
                break
            time.sleep(0.05)

        if not stopped:
            self._safe_stop(driver)

        result.position = actual_m
        result.stalled = stalled
        # control_msgs defines effort in Newtons; the PGEA only exposes motor
        # current, so report 0.0 rather than a unit-mismatched raw register.
        result.effort = 0.0

        if alarm:
            goal_handle.abort()
            return result
        if stalled and target_m > 1e-6:
            # Closing stopped on an object: this is a successful grasp even
            # though the full close target was not reached.
            result.reached_goal = True
            goal_handle.succeed()
            self.get_logger().info(
                f"Gripper stopped on object at {actual_m:.3f} m (stalled)"
            )
            return result
        if stalled:
            self.get_logger().error(
                "Gripper remained obstructed while opening; open goal aborted"
            )
            goal_handle.abort()
            return result
        if stopped and abs(actual_m - target_m) <= tolerance:
            result.reached_goal = True
            goal_handle.succeed()
            self.get_logger().info(
                f"Gripper in position: {actual_m:.3f} m (target {target_m:.3f} m)"
            )
            return result
        reason = (
            f"stopped at {actual_m:.3f} m, {abs(actual_m - target_m):.4f} m from target"
            if stopped
            else f"timed out after {self._move_timeout:.0f}s at {actual_m:.3f} m"
        )
        self.get_logger().error(f"GripperCommand did not reach goal: {reason}")
        goal_handle.abort()
        return result

    # ── feedback timer ────────────────────────────────────────────────
    def _feedback_tick(self) -> None:
        if self._driver is None or not self._driver.connected:
            self._maybe_reconnect()
            if self._driver is None or not self._driver.connected:
                # Do not publish last-known positions with a fresh timestamp:
                # MoveIt must treat disconnected finger feedback as stale.
                return
        driver = self._driver
        try:
            init_state, grip, position_pm, alarm = self._read_state(driver)
        except TransportError as exc:
            self._read_failures += 1
            self.get_logger().warn(
                f"PGEA feedback read failed ({self._read_failures}x): {exc}",
                throttle_duration_sec=5.0,
            )
            if self._read_failures >= 3:
                self.get_logger().error(
                    "PGEA link lost — closing connection, will retry"
                )
                self._close_driver()
            # Do not refresh stale joint feedback after a Modbus read failure.
            return
        self._read_failures = 0
        self._last_init_state = init_state
        position_m = permille_to_m(position_pm)
        self._last_position_m = position_m

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # right_finger_joint mimics left with multiplier +1 (cr5_pgea SRDF/URDF).
        msg.name = ["left_finger_joint", "right_finger_joint"]
        msg.position = [position_m, position_m]
        self._state_pub.publish(msg)

        if alarm:
            self.get_logger().error(
                f"PGEA alarm: {ALARMS.get(alarm, (f'code 0x{alarm:02X}', ''))[0]} — "
                f"{ALARMS.get(alarm, ('', 'Clear via the controller UI/app.'))[1]}",
                throttle_duration_sec=10.0,
            )
        elif init_state != 1:
            self.get_logger().warn(
                f"PGEA state: {INIT_STATES.get(init_state, init_state)} / "
                f"grip: {GRIP_STATES.get(grip, grip)}",
                throttle_duration_sec=10.0,
            )

    def shutdown(self) -> None:
        # Try to stop any commanded movement while the transport is still
        # available; closing RS-485/Dashboard alone is not a motion stop.
        driver = self._driver
        if driver is not None and driver.connected:
            self._safe_stop(driver)
        self._jog_deadline = None
        self._close_driver()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PGEAGripperNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
