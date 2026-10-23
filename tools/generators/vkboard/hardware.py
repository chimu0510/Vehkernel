"""Ports, board devices, storage, timing and board default parameters (ADR-0011).

- ``serial``: logical ports such as SERIAL1 with their TX/RX pins, default
  use and optional default baud rate, which the ``SERIALn_FUNCTION`` and
  ``SERIALn_BAUD`` parameters change, and the download circuit of a port
  whose adapter drives BOOT0 and NRST (ADR-0023).
- ``devices``: board devices with kind, type, peripheral bus, pins, I2C
  address and mounting rotation (ADR-0014, ADR-0015).
- ``storage``: storage devices that log sinks can use (ADR-0019).
- ``timing``: the board's timing preset (ADR-0015).
- ``defaults``: any other board default parameters (ADR-0012).
- ``actuators``: actuator outputs by number, each a motor driver's PWM
  channel and two direction lines with the quadrature encoder paired with it,
  and optionally the encoder's counts per wheel turn for the stock motors,
  which becomes the ``MOTn_CPR`` board default (ADR-0011).
- ``revision``: the revision-ID pins, least significant bit first, and the
  value they must read at boot (ADR-0010).
- ``actuator_power``: on boards that have one, the input that reads whether
  the actuators' power circuit is closed, and its powered level (ADR-0005).
- ``user_key``: on boards that have one, the push button that operates the
  local display and the local menu, and the level it reads while pressed
  (ADR-0022).
"""

from __future__ import annotations

import re
from dataclasses import dataclass

from .diagnostics import Diagnostics, Loc

SERIAL_KEYS = {"port", "tx", "rx", "function", "baud", "download"}
# DTR_RTS: RTS asserted holds BOOT0 high; DTR released with RTS asserted holds NRST low.
DOWNLOAD_CIRCUITS = ("DTR_RTS",)
SERIAL_FUNCTIONS = ("NONE", "LINK", "RC_INPUT", "LOG")
DEVICE_KEYS = {"device", "kind", "type", "bus", "pins", "address", "rotation"}
DEVICE_BUSES = ("I2C", "SOFT_I2C", "SPI", "SOFT_SPI")
STORAGE_KEYS = {"storage", "bus"}
STORAGE_KINDS = {"SD_CARD": ("SDIO", "SPI"), "SPI_NOR_FLASH": ("SPI",)}
TIMING_KEYS = {"trigger", "base_rate_hz", "sampling", "dividers"}
ACTUATOR_KEYS = {"output", "pwm", "in1", "in2", "encoder", "cpr"}
# The most actuator outputs a board may have: the length of the actuator topics' arrays.
MAX_ACTUATORS = 4
REVISION_KEYS = {"pins", "value"}
ACTUATOR_POWER_KEYS = {"pin", "powered"}
POWERED_LEVELS = ("HIGH", "LOW")
USER_KEY_KEYS = {"pin", "pressed"}
TIMING_TRIGGERS = ("IMU_EVENT", "TIMER")
TIMING_SAMPLING = ("PER_SAMPLE", "FIFO")
TIMING_DIVIDERS = {"estimator": "CTRL_DIV_EST", "rate": "CTRL_DIV_RATE",
                   "attitude": "CTRL_DIV_ATT", "velocity": "CTRL_DIV_VEL"}
ROTATIONS = ("NONE", "YAW_45", "YAW_90", "YAW_135", "YAW_180", "YAW_225", "YAW_270", "YAW_315",
             "ROLL_180", "ROLL_180_YAW_45", "ROLL_180_YAW_90", "ROLL_180_YAW_135", "PITCH_180",
             "ROLL_180_YAW_225", "ROLL_180_YAW_270", "ROLL_180_YAW_315", "ROLL_90", "ROLL_270",
             "PITCH_90", "PITCH_270")

_ID_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")
_PORT_RE = re.compile(r"^SERIAL(\d+)$")
_PARAM_RE = re.compile(r"^[A-Z][A-Z0-9_]{0,15}$")
_SERIAL_FUNC_RE = re.compile(r"^(U(?:S)?ART\d+)_(TX|RX)$")
_TIMER_CHANNEL_RE = re.compile(r"^TIM(\d+)_CH(\d)$")
# Pin functions each bus needs, in the order the device's pins list them.
_BUS_SIGNALS = {"I2C": ("SCL", "SDA"), "SPI": ("SCK", "MISO", "MOSI")}


@dataclass(frozen=True)
class SerialPort:
    port: str
    peripheral: str
    tx: str | None
    rx: str | None
    function: str
    baud: int | None = None
    download: str | None = None


@dataclass(frozen=True)
class Device:
    device: str
    kind: str
    type: str
    bus: str
    peripheral: str | None  # hardware I2C/SPI instance, None for SOFT_I2C and SOFT_SPI
    pins: tuple[str, ...]
    address: int | None
    rotation: str


@dataclass(frozen=True)
class Storage:
    storage: str
    bus: str


@dataclass(frozen=True)
class Actuator:
    output: int
    pwm: str  # pin roles
    in1: str
    in2: str
    encoder: tuple[str, str]  # channel A on CH1, channel B on CH2 of one timer
    pwm_timer: int
    pwm_channel: int
    encoder_timer: int
    cpr: float | None = None


@dataclass(frozen=True)
class Revision:
    pins: tuple[str, ...]  # pin roles, least significant bit first
    value: int


@dataclass(frozen=True)
class ActuatorPower:
    pin: str  # pin role
    powered: str  # level read while powered: HIGH or LOW


@dataclass(frozen=True)
class UserKey:
    pin: str  # pin role
    pressed: str  # level read while pressed: HIGH or LOW


@dataclass(frozen=True)
class Hardware:
    serial: tuple[SerialPort, ...] = ()
    devices: tuple[Device, ...] = ()
    storage: tuple[Storage, ...] = ()
    defaults: tuple[tuple[str, object], ...] = ()  # sorted by name
    revision: Revision | None = None
    actuator_power: ActuatorPower | None = None
    actuators: tuple[Actuator, ...] = ()
    user_key: UserKey | None = None


def _keys(entry: object, allowed: set, required: set, what: str, loc: Loc, diag: Diagnostics) -> bool:
    if not isinstance(entry, dict):
        diag.error(f"each {what} entry must be an inline table", loc)
        return False
    ok = True
    for key in sorted(set(entry) - allowed):
        diag.error(f"unknown key '{key}' in {what} entry", loc)
        ok = False
    for key in sorted(required - set(entry)):
        diag.error(f"{what} entry is missing '{key}'", loc)
        ok = False
    return ok


def _serial(entries, roles: dict, diag: Diagnostics) -> list[SerialPort]:
    ports, seen = [], {}
    for entry, loc in entries:
        if not _keys(entry, SERIAL_KEYS, {"port", "function"}, "serial", loc, diag):
            continue
        port, function = entry["port"], entry["function"]
        if not isinstance(port, str) or not _PORT_RE.match(port):
            diag.error(f"serial port '{port}' must be named SERIAL<n>", loc)
            continue
        if port in seen:
            diag.error(f"serial port {port} is already defined ({seen[port].refer(loc)})", loc)
            continue
        seen[port] = loc
        if function not in SERIAL_FUNCTIONS:
            diag.error(f"default function '{function}' of {port} must be one of: {', '.join(SERIAL_FUNCTIONS)}", loc)
            continue
        if "tx" not in entry and "rx" not in entry:
            diag.error(f"serial port {port} needs a 'tx' or 'rx' pin role", loc)
            continue
        baud = entry.get("baud")
        if baud is not None and (not isinstance(baud, int) or isinstance(baud, bool) or not 1200 <= baud <= 10_000_000):
            diag.error(f"baud of {port} must be an integer from 1200 to 10000000, got {baud!r}", loc)
            continue
        download = entry.get("download")
        if download is not None and download not in DOWNLOAD_CIRCUITS:
            # The port itself is fine, so a bootloader listing it reports nothing more.
            diag.error(f"download circuit of {port} must be one of: {', '.join(DOWNLOAD_CIRCUITS)}, "
                       f"got {download!r}", loc)
            download = None
        peripherals, ok = set(), True
        for direction in ("tx", "rx"):
            role = entry.get(direction)
            if role is None:
                continue
            pin = roles.get(role)
            match = _SERIAL_FUNC_RE.match(pin.func) if pin else None
            if match is None or match.group(2) != direction.upper():
                diag.error(f"{direction} of {port} must be a pin role with a U(S)ART {direction.upper()} "
                           f"function, not '{role}'", loc)
                ok = False
                continue
            peripherals.add(match.group(1))
        if ok and len(peripherals) > 1:
            diag.error(f"tx and rx of {port} use different peripherals: {', '.join(sorted(peripherals))}", loc)
            ok = False
        if ok:
            ports.append(SerialPort(port, peripherals.pop(), entry.get("tx"), entry.get("rx"), function, baud,
                                    download))
    return ports


def _device(entry: dict, loc: Loc, roles: dict, diag: Diagnostics) -> Device | None:
    name, kind, dtype, bus = entry["device"], entry["kind"], entry["type"], entry["bus"]
    for label, value in (("device", name), ("kind", kind), ("type", dtype)):
        if not isinstance(value, str) or not _ID_RE.match(value):
            diag.error(f"device {label} '{value}' must be upper case letters, digits and underscores", loc)
            return None
    if bus not in DEVICE_BUSES:
        diag.error(f"bus of device {name} must be one of: {', '.join(DEVICE_BUSES)}", loc)
        return None
    pins = entry["pins"]
    if not isinstance(pins, list) or not all(isinstance(r, str) for r in pins):
        diag.error(f"pins of device {name} must be a list of pin roles", loc)
        return None
    missing = [r for r in pins if r not in roles]
    if missing:
        diag.error(f"device {name} uses undefined pin roles: {', '.join(missing)}", loc)
        return None

    peripheral = None
    if bus == "SOFT_I2C":
        if len(pins) != 2 or any(roles[r].func != "GPIO_OUTPUT" for r in pins):
            diag.error(f"SOFT_I2C device {name} needs two GPIO_OUTPUT pins: SCL, SDA", loc)
            return None
    elif bus == "SOFT_SPI":
        # Write-only: a display's clock, data, reset and data/command lines (ADR-0022).
        if kind != "DISPLAY":
            diag.error(f"SOFT_SPI carries only DISPLAY devices, not {kind} device {name}", loc)
            return None
        if len(pins) != 4 or any(roles[r].func != "GPIO_OUTPUT" for r in pins):
            diag.error(f"SOFT_SPI device {name} needs four GPIO_OUTPUT pins: clock, data, reset, data/command", loc)
            return None
    else:
        signals = _BUS_SIGNALS[bus]
        expected = len(signals) + (1 if bus == "SPI" else 0)
        hardware = pins[:len(signals)]
        instances = {roles[r].func.rsplit("_", 1)[0] for r in hardware}
        functions_ok = len(pins) == expected and all(
            roles[r].func.startswith(bus) and roles[r].func.rsplit("_", 1)[1] == s
            for r, s in zip(hardware, signals))
        cs_ok = bus != "SPI" or (len(pins) == expected and roles[pins[-1]].func == "GPIO_OUTPUT")
        if not functions_ok or not cs_ok or len(instances) != 1:
            order = ", ".join(signals) + (", CS (GPIO_OUTPUT)" if bus == "SPI" else "")
            diag.error(f"{bus} device {name} needs pins of one {bus} peripheral in the order {order}", loc)
            return None
        peripheral = instances.pop()

    address = entry.get("address")
    if bus in ("I2C", "SOFT_I2C"):
        if not isinstance(address, int) or isinstance(address, bool) or not 0x08 <= address <= 0x77:
            diag.error(f"I2C device {name} needs a 7-bit 'address' between 0x08 and 0x77", loc)
            return None
    elif address is not None:
        diag.error(f"{bus} device {name} takes no 'address'", loc)
        return None
    rotation = entry.get("rotation", "NONE")
    if rotation not in ROTATIONS:
        diag.error(f"rotation '{rotation}' of device {name} is not one of the known rotations", loc)
        return None
    return Device(name, kind, dtype, bus, peripheral, tuple(pins), address, rotation)


def _devices(entries, roles: dict, diag: Diagnostics) -> list[Device]:
    devices, seen = [], {}
    for entry, loc in entries:
        if not _keys(entry, DEVICE_KEYS, DEVICE_KEYS - {"address", "rotation"}, "device", loc, diag):
            continue
        name = entry["device"]
        if name in seen:
            diag.error(f"device {name} is already defined ({seen[name].refer(loc)})", loc)
            continue
        seen[name] = loc
        device = _device(entry, loc, roles, diag)
        if device:
            devices.append(device)
    return devices


def _storage(entries, diag: Diagnostics) -> list[Storage]:
    result, seen = [], {}
    for entry, loc in entries:
        if not _keys(entry, STORAGE_KEYS, STORAGE_KEYS, "storage", loc, diag):
            continue
        kind, bus = entry["storage"], entry["bus"]
        if kind not in STORAGE_KINDS:
            diag.error(f"storage '{kind}' must be one of: {', '.join(STORAGE_KINDS)}", loc)
            continue
        if kind in seen:
            diag.error(f"storage {kind} is already defined ({seen[kind].refer(loc)})", loc)
            continue
        seen[kind] = loc
        if bus not in STORAGE_KINDS[kind]:
            diag.error(f"storage {kind} connects over {' or '.join(STORAGE_KINDS[kind])}, not '{bus}'", loc)
            continue
        result.append(Storage(kind, bus))
    return result


def _timing(timing: object, loc: Loc, diag: Diagnostics) -> dict[str, object]:
    if timing is None:
        return {}
    if not isinstance(timing, dict):
        diag.error("'timing' must be an inline table", loc)
        return {}
    for key in sorted(set(timing) - TIMING_KEYS):
        diag.error(f"unknown key '{key}' in timing", loc)
    params: dict[str, object] = {}
    if "trigger" in timing:
        if timing["trigger"] not in TIMING_TRIGGERS:
            diag.error(f"timing trigger must be one of: {', '.join(TIMING_TRIGGERS)}", loc)
        params["CTRL_TRIGGER"] = timing["trigger"]
    if "base_rate_hz" in timing:
        rate = timing["base_rate_hz"]
        if not isinstance(rate, int) or isinstance(rate, bool) or not 1 <= rate <= 8000:
            diag.error("timing base_rate_hz must be an integer from 1 to 8000", loc)
        params["CTRL_RATE_HZ"] = rate
    if "sampling" in timing:
        if timing["sampling"] not in TIMING_SAMPLING:
            diag.error(f"timing sampling must be one of: {', '.join(TIMING_SAMPLING)}", loc)
        params["IMU_SAMPLING"] = timing["sampling"]
    dividers = timing.get("dividers", {})
    if not isinstance(dividers, dict):
        diag.error("timing dividers must be an inline table", loc)
        dividers = {}
    for layer, value in dividers.items():
        if layer not in TIMING_DIVIDERS:
            diag.error(f"unknown timing divider '{layer}' (expected: {', '.join(TIMING_DIVIDERS)})", loc)
        elif not isinstance(value, int) or isinstance(value, bool) or value < 1:
            diag.error(f"timing divider '{layer}' must be an integer of at least 1", loc)
        else:
            params[TIMING_DIVIDERS[layer]] = value
    return params


def _timer_channel(role: object, roles: dict) -> tuple[int, int] | None:
    pin = roles.get(role) if isinstance(role, str) else None
    match = _TIMER_CHANNEL_RE.match(pin.func) if pin is not None else None
    return (int(match.group(1)), int(match.group(2))) if match else None


def _actuator(entry: dict, loc: Loc, roles: dict, diag: Diagnostics) -> Actuator | None:
    output = entry["output"]
    if not isinstance(output, int) or isinstance(output, bool) or not 1 <= output <= MAX_ACTUATORS:
        diag.error(f"actuator output must be a number from 1 to {MAX_ACTUATORS}, got {output!r}", loc)
        return None
    pwm = _timer_channel(entry["pwm"], roles)
    if pwm is None:
        diag.error(f"pwm of output {output} must be a pin role with a timer channel function, "
                   f"not {entry['pwm']!r}", loc)
        return None
    for line in ("in1", "in2"):
        role = entry[line]
        pin = roles.get(role) if isinstance(role, str) else None
        if pin is None or pin.kind != "gpio_output":
            diag.error(f"{line} of output {output} must be a pin role with the GPIO_OUTPUT function, "
                       f"not {role!r}", loc)
            return None
    encoder = entry["encoder"]
    channels = [_timer_channel(role, roles) for role in encoder] if isinstance(encoder, list) else []
    if len(channels) != 2 or None in channels or channels[0][0] != channels[1][0] \
            or [c[1] for c in channels] != [1, 2]:
        diag.error(f"encoder of output {output} must be two pin roles on CH1 and CH2 of one timer, "
                   f"not {encoder!r}", loc)
        return None
    cpr = entry.get("cpr")
    if cpr is not None and (not isinstance(cpr, (int, float)) or isinstance(cpr, bool) or cpr <= 0):
        diag.error(f"cpr of output {output} must be a positive number of counts per wheel turn", loc)
        return None
    return Actuator(output, entry["pwm"], entry["in1"], entry["in2"], (encoder[0], encoder[1]),
                    pwm[0], pwm[1], channels[0][0], cpr)


def _actuators(entries, roles: dict, diag: Diagnostics) -> list[Actuator]:
    actuators, seen = [], {}
    for entry, loc in entries:
        if not _keys(entry, ACTUATOR_KEYS, ACTUATOR_KEYS - {"cpr"}, "actuator", loc, diag):
            continue
        actuator = _actuator(entry, loc, roles, diag)
        if actuator is None:
            continue
        if actuator.output in seen:
            diag.error(f"actuator output {actuator.output} is already defined ({seen[actuator.output].refer(loc)})", loc)
            continue
        seen[actuator.output] = loc
        actuators.append(actuator)
    numbers = sorted(seen)
    if numbers != list(range(1, len(numbers) + 1)):
        diag.error(f"actuator outputs must be numbered from 1 without gaps, got {numbers}",
                   entries[0][1] if entries else Loc(""))
    return sorted(actuators, key=lambda a: a.output)


def _gpio_input(role: object, what: str, roles: dict, loc: Loc, diag: Diagnostics) -> bool:
    pin = roles.get(role) if isinstance(role, str) else None
    if pin is None or pin.kind != "gpio_input":
        diag.error(f"{what} must be a pin role with the GPIO_INPUT function, not {role!r}", loc)
        return False
    return True


def _revision(entry: object, loc: Loc, roles: dict, diag: Diagnostics) -> Revision | None:
    if entry is None:
        return None
    if not _keys(entry, REVISION_KEYS, REVISION_KEYS, "revision", loc, diag):
        return None
    pins, value = entry["pins"], entry["value"]
    if not isinstance(pins, list) or not pins:
        diag.error("'pins' of the revision must be a non-empty array of pin roles", loc)
        return None
    if not all(_gpio_input(role, "each revision pin", roles, loc, diag) for role in pins):
        return None
    if len(set(pins)) != len(pins):
        diag.error("a revision pin is listed twice", loc)
        return None
    if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value < (1 << len(pins)):
        diag.error(f"the revision value must be an integer from 0 to {(1 << len(pins)) - 1} "
                   f"for {len(pins)} pins, got {value!r}", loc)
        return None
    return Revision(tuple(pins), value)


def _actuator_power(entry: object, loc: Loc, roles: dict, diag: Diagnostics) -> ActuatorPower | None:
    if entry is None:
        return None
    if not _keys(entry, ACTUATOR_POWER_KEYS, ACTUATOR_POWER_KEYS, "actuator_power", loc, diag):
        return None
    if not _gpio_input(entry["pin"], "the actuator power pin", roles, loc, diag):
        return None
    if entry["powered"] not in POWERED_LEVELS:
        diag.error(f"the powered level must be one of: {', '.join(POWERED_LEVELS)}, got {entry['powered']!r}", loc)
        return None
    return ActuatorPower(entry["pin"], entry["powered"])


def _user_key(entry: object, loc: Loc, roles: dict, diag: Diagnostics) -> UserKey | None:
    if entry is None:
        return None
    if not _keys(entry, USER_KEY_KEYS, USER_KEY_KEYS, "user_key", loc, diag):
        return None
    if not _gpio_input(entry["pin"], "the user key pin", roles, loc, diag):
        return None
    if entry["pressed"] not in POWERED_LEVELS:
        diag.error(f"the pressed level must be one of: {', '.join(POWERED_LEVELS)}, got {entry['pressed']!r}", loc)
        return None
    return UserKey(entry["pin"], entry["pressed"])


def check_hardware(serial, devices, storage, timing, defaults, roles: dict, top: Loc,
                   timing_loc: Loc, defaults_loc: Loc, diag: Diagnostics,
                   revision: tuple[object, Loc] | None = None,
                   actuator_power: tuple[object, Loc] | None = None,
                   actuators=(),
                   user_key: tuple[object, Loc] | None = None) -> Hardware:
    """Validate the hardware sections; ``roles`` maps default role names to pins."""
    ports = _serial(serial, roles, diag)
    params: dict[str, object] = {f"{p.port}_FUNCTION": p.function for p in ports}
    params.update({f"{p.port}_BAUD": p.baud for p in ports if p.baud is not None})
    outputs = _actuators(actuators, roles, diag)
    params.update({f"MOT{a.output}_CPR": a.cpr for a in outputs if a.cpr is not None})
    for name, value in _timing(timing, timing_loc, diag).items():
        params[name] = value

    if not isinstance(defaults, dict):
        diag.error("'defaults' must be an inline table of parameter names and values", top)
        defaults = {}
    for name, value in defaults.items():
        if not _PARAM_RE.match(name):
            diag.error(f"parameter name '{name}' must be upper case and at most 16 characters", defaults_loc)
        elif name in params:
            diag.error(f"parameter {name} is already set by the serial, timing or actuators section", defaults_loc)
        elif isinstance(value, (dict, list)):
            diag.error(f"default of parameter {name} must be a number, boolean or string", defaults_loc)
        else:
            params[name] = value
    return Hardware(tuple(ports), tuple(_devices(devices, roles, diag)), tuple(_storage(storage, diag)),
                    tuple(sorted(params.items())),
                    _revision(*(revision or (None, top)), roles, diag),
                    _actuator_power(*(actuator_power or (None, top)), roles, diag),
                    tuple(outputs),
                    _user_key(*(user_key or (None, top)), roles, diag))
