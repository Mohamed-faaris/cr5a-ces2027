"""PGEA register map and display labels, transcribed from PGEA V4.0 manual."""
REGISTERS = {
    "Initialize": 0x0100, "Force": 0x0101, "Position": 0x0103,
    "Speed": 0x0104, "Jog": 0x010A,
    "Initialization status": 0x0200, "Grip status": 0x0201,
    "Actual position": 0x0202, "Motor current feedback": 0x0204,
    "Error feedback": 0x0205, "Reset status": 0x0206,
    "Motor temperature": 0x020F,
    "Save parameters": 0x0300, "Initialization direction": 0x0301,
    "Device ID": 0x0302, "Baud-rate code": 0x0303,
    "Stop bits": 0x0304, "Parity": 0x0305,
    "I/O group test": 0x0400, "I/O mode switch": 0x0402,
    "I/O input polarity": 0x0403, "I/O output polarity": 0x0404,
    "I/O group 1 position": 0x0405, "I/O group 1 force": 0x0406,
    "I/O group 1 speed": 0x0407, "I/O group 2 position": 0x0408,
    "I/O group 2 force": 0x0409, "I/O group 2 speed": 0x040A,
    "I/O group 3 position": 0x040B, "I/O group 3 force": 0x040C,
    "I/O group 3 speed": 0x040D, "I/O group 4 position": 0x040E,
    "I/O group 4 force": 0x040F, "I/O group 4 speed": 0x0410,
    "I/O input filter": 0x0411, "I/O input status": 0x0412,
    "I/O output status": 0x0413, "Clear error": 0x0503,
    # PGEA manual section 3 mentions 0x0504 for power-on auto-init, while
    # the configuration register table is inconsistent. This is intentionally
    # NOT included in the documented writable list below.
    "Power-on auto-init (verify firmware)": 0x0504,
}

BAUD_CODES = {0: 115200, 1: 57600, 2: 38400, 3: 19200, 4: 9600, 5: 4800}
INIT_STATES = {
    0: "Not initialized", 1: "Initialized", 2: "Initializing",
    0xFFFF: "Calibration error",
}
GRIP_STATES = {
    0: "Moving", 1: "Position reached", 2: "Object caught",
    3: "Object dropped", 0xFFFF: "Initialization incomplete",
}

READ_ONLY_REGS = {
    0x0200: "Initialization status", 0x0201: "Grip status",
    0x0202: "Actual position", 0x0204: "Motor current feedback",
    0x0205: "Error feedback", 0x0206: "Reset status",
    0x020F: "Motor temperature", 0x0412: "I/O input status",
    0x0413: "I/O output status",
}

# Used to constrain writes from the generic register editor. Unknown registers
# are blocked by default unless the operator confirms an advanced write.
WRITABLE_REGS = {
    0x0101: ("Force (%)", 20, 100),
    0x0104: ("Speed (%)", 1, 100),
    0x0300: ("Save parameters (1)", 0, 1),
    0x0301: ("Initialization direction (0 open, 1 close)", 0, 1),
    0x0302: ("Device ID", 1, 247),
    0x0303: ("Baud-rate code (0=115200 ... 5=4800)", 0, 5),
    0x0304: ("Stop bits (0=1, 1=2)", 0, 1),
    0x0305: ("Parity (0 none, 1 odd, 2 even)", 0, 2),
    0x0400: ("I/O group test (1-4)", 1, 4),
    0x0402: ("I/O mode switch (0 off, 1 on)", 0, 1),
    0x0403: ("I/O input polarity (0 low, 1 high)", 0, 1),
    0x0404: ("I/O output polarity (0 low, 1 high)", 0, 1),
    0x0411: ("I/O input filter (ms)", 0, 32767),
    0x0503: ("Clear error (1)", 0, 1),
    # 0x0504 deliberately withheld because of manual ambiguity.
}

ALARMS = {
    0x00: ("No error", "No alarm reported."),
    0x01: ("Undervoltage", "Manual: control voltage below 15 V. Check the regulated 24 V supply and wiring."),
    0x02: ("Overvoltage", "Manual: voltage above 28 V. Check supply voltage and remote-sensing arrangement."),
    0x03: ("Overcurrent", "Check supply stability and wiring. If persistent, stop and contact the manufacturer."),
    0x04: ("Overheating", "Manual: motor NTC above 90 °C. Avoid prolonged stalls and allow cooling."),
    0x05: ("Motor disconnection (basic map)", "Inspect motor/cable connections. The detailed table labels code 0x0A as motor phase loss."),
    0x08: ("Overload (basic map)", "Check for mechanical obstruction and load. Isolate power before inspecting mechanics."),
    0x0A: ("Motor phase loss", "Inspect wiring/connectors; clear only after the cause has been addressed."),
    0x0B: ("Overspeed", "Check external forces and control behavior."),
    0x0F: ("Startup error (basic map)", "Review wiring and manufacturer guidance before power cycling."),
    0x20: ("Encoder status error", "Inspect cable, interference, encoder alignment and shaft with power isolated."),
}

IO_STATUS = {
    0x0412: {0: "Input 1 OFF, Input 2 OFF", 1: "Input 1 ON, Input 2 OFF", 2: "Input 1 OFF, Input 2 ON", 3: "Input 1 ON, Input 2 ON"},
    0x0413: {0: "Moving", 1: "In position", 2: "Stalled/object detected", 3: "Falling/object dropped"},
}
