"""PGEA register-level logic independent of the physical transport."""
from __future__ import annotations
import time
from .register_map import INIT_STATES, GRIP_STATES
from .transport import TransportError

# Grip status (0x0201) values used by the force-limited close loop.
GRIP_MOVING = 0
GRIP_IN_POSITION = 1
GRIP_OBJECT_CAUGHT = 2

class PGEADriver:
    def __init__(self, transport):
        self.transport = transport

    @property
    def connected(self):
        return self.transport.connected

    @property
    def last_tx(self):
        return self.transport.last_tx

    @property
    def last_rx(self):
        return self.transport.last_rx

    def connect(self):
        self.transport.connect()
        return self

    def close(self):
        self.transport.close()

    def read_register(self, address: int) -> int:
        return self.transport.read_register(address)

    def read_registers(self, address: int, count: int = 1) -> list[int]:
        return self.transport.read_registers(address, count)

    def write_register(self, address: int, value: int) -> int:
        return self.transport.write_register(address, value)

    def initialize(self, full_calibration: bool = False):
        # PGEA manual: 0x01 direction-calibrates; 0xA5 performs full close-open calibration.
        return self.write_register(0x0100, 0x00A5 if full_calibration else 0x0001)

    def _require_initialized(self):
        state = self.read_register(0x0200)
        if state != 1:
            raise TransportError(
                f"Motion blocked: PGEA initialization state is {INIT_STATES.get(state, hex(state))}. "
                "Run initialization and verify feedback first."
            )

    def move(self, position: int, force: int, speed: int):
        if not 0 <= position <= 1000:
            raise ValueError("Position must be 0–1000 per mille")
        if not 20 <= force <= 100:
            raise ValueError("Force must be 20–100 percent")
        if not 1 <= speed <= 100:
            raise ValueError("Speed must be 1–100 percent")
        self._require_initialized()
        # Prepare force and speed, then commit the position command that causes motion.
        self.write_register(0x0101, force)
        self.write_register(0x0104, speed)
        return self.write_register(0x0103, position)

    def jog(self, direction: int):
        if direction not in (-1, 0, 1):
            raise ValueError("Jog direction must be -1, 0 or 1")
        if direction:
            self._require_initialized()
        return self.write_register(0x010A, 0xFFFF if direction == -1 else direction)

    def _stop_motion(self):
        """Jog-stop. Deliberately not initialization-gated: stopping must always work."""
        return self.jog(0)

    def close_until_force(self, force_limit: int, speed: int = 20,
                          close_position: int = 1000, timeout: float = 20.0,
                          poll_interval: float = 0.05, max_current=None,
                          progress=None, cancel=None) -> dict:
        """Close the jaws at a fixed force envelope and stop once that force is on the object.

        The PGEA applies the force written to 0x0101 (20-100 %) as its gripping
        force envelope. This routine writes the caller's force limit, commands
        closing travel toward `close_position` (1000 per mille = fully closed),
        and polls feedback until one of these outcomes:

            object_caught     grip status 0x0201 = 2: the jaws stopped on the
                              object and the commanded force limit is now the
                              force applied to it. No stop command is sent,
                              because stopping would be a no-op and the gripper
                              holds the object by itself at that envelope.
            position_reached  grip status 0x0201 = 1: the close target was met
                              without catching anything, so the force limit was
                              never loaded on an object.
            current_limit     motor current 0x0204 reached `max_current` (raw
                              register counts; optional guard, None disables it).
            alarm             non-zero alarm feedback 0x0205.
            timeout           `timeout` seconds elapsed while still moving.
            canceled          the `cancel` callback returned True.
            read_error        feedback could not be read several times running.

        `progress` is called with a snapshot dict on every poll, and `cancel`
        is polled before each read so an operator stop takes effect within one
        poll interval. A stop is always commanded on every non-success exit.
        """
        if not 20 <= force_limit <= 100:
            raise ValueError("Force limit must be 20–100 percent")
        if not 1 <= speed <= 100:
            raise ValueError("Speed must be 1–100 percent")
        if not 0 <= close_position <= 1000:
            raise ValueError("Close position must be 0–1000 per mille")
        if timeout <= 0:
            raise ValueError("Timeout must be positive seconds")
        if max_current is not None and max_current <= 0:
            raise ValueError("Motor-current stop threshold must be positive, or None to disable")
        poll = max(0.02, float(poll_interval))
        self._require_initialized()
        # Same commit order as move(): prepare force and speed first, then the
        # position command that actually starts the closing travel.
        self.write_register(0x0101, force_limit)
        self.write_register(0x0104, speed)
        self.write_register(0x0103, close_position)

        result = {
            "outcome": "closing", "force": force_limit, "speed": speed,
            "close_position": close_position, "position": None, "grip": None,
            "current": None, "alarm": 0, "elapsed": 0.0, "error": None,
        }
        started = time.monotonic()
        deadline = started + timeout
        read_failures = 0
        while time.monotonic() < deadline:
            if cancel is not None and cancel():
                self._stop_motion()
                result.update(outcome="canceled", elapsed=time.monotonic() - started)
                return result
            try:
                grip, position = self.read_registers(0x0201, 2)
                current = self.read_register(0x0204)
                alarm = self.read_register(0x0205)
            except TransportError as exc:
                # Transient bus noise should not abort a grip in progress, but a
                # dead link must stop the jaws rather than let them keep closing.
                read_failures += 1
                result["error"] = str(exc)
                result["elapsed"] = time.monotonic() - started
                if read_failures >= 5:
                    self._stop_motion()
                    result["outcome"] = "read_error"
                    return result
                time.sleep(poll)
                continue
            read_failures = 0
            result.update(position=position, grip=grip, current=current,
                          alarm=alarm, error=None,
                          elapsed=time.monotonic() - started)
            if progress is not None:
                progress(dict(result))
            if alarm:
                self._stop_motion()
                result["outcome"] = "alarm"
                return result
            if max_current is not None and current >= max_current:
                self._stop_motion()
                result["outcome"] = "current_limit"
                return result
            if grip == GRIP_OBJECT_CAUGHT:
                result["outcome"] = "object_caught"
                return result
            if grip == GRIP_IN_POSITION:
                result["outcome"] = "position_reached"
                return result
            time.sleep(poll)
        self._stop_motion()
        result["elapsed"] = time.monotonic() - started
        result["outcome"] = "timeout"
        return result

    def test_io_group(self, group: int):
        if group not in (1, 2, 3, 4):
            raise ValueError("I/O group must be 1–4")
        self._require_initialized()
        return self.write_register(0x0400, group)

    def safe_write_register(self, address: int, value: int):
        """Apply GUI guardrails even when writing from the raw register editor."""
        if address == 0x0100:
            if value == 1:
                return self.initialize(False)
            if value == 0x00A5:
                return self.initialize(True)
            raise ValueError("Initialization register accepts 1 or 0xA5")
        if address == 0x0101:
            if not 20 <= value <= 100:
                raise ValueError("Force must be 20–100 percent")
            self._require_initialized()
        elif address == 0x0103:
            if not 0 <= value <= 1000:
                raise ValueError("Position must be 0–1000 per mille")
            self._require_initialized()
        elif address == 0x0104:
            if not 1 <= value <= 100:
                raise ValueError("Speed must be 1–100 percent")
            self._require_initialized()
        elif address == 0x010A:
            if value == 0xFFFF:
                return self.jog(-1)
            if value in (0, 1):
                return self.jog(value)
            raise ValueError("Jog register accepts 0xFFFF (-1), 0 (stop), or 1 (forward)")
        elif address == 0x0400:
            if not 1 <= value <= 4:
                raise ValueError("I/O group test must be 1–4")
            self._require_initialized()
        return self.write_register(address, value)

    def save_parameters(self):
        return self.write_register(0x0300, 1)

    def read_snapshot(self) -> dict:
        values = {}
        for key, address in (
            ("init", 0x0200), ("grip", 0x0201), ("position", 0x0202),
            ("current", 0x0204), ("error", 0x0205),
        ):
            values[key] = self.read_register(address)
        try:
            values["temperature"] = self.read_register(0x020F)
        except Exception as exc:
            # Some firmware revisions do not expose this optional register.
            values["temperature"] = None
            values["temperature_note"] = str(exc)
        for key, address in (("io_input", 0x0412), ("io_output", 0x0413)):
            try:
                values[key] = self.read_register(address)
            except Exception:
                values[key] = None
        return values

    @staticmethod
    def state_text(snapshot: dict) -> dict:
        return {
            "init": INIT_STATES.get(snapshot.get("init"), f"Unknown ({snapshot.get('init')})"),
            "grip": GRIP_STATES.get(snapshot.get("grip"), f"Unknown ({snapshot.get('grip')})"),
        }
