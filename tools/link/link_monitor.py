#!/usr/bin/env python3
"""Link monitor: watch the topics a Vehkernel Link sends (spec 0003).

Connects over UDP (SITL's default) or a serial port (a board's USB serial
port, or SITL's pty), performs the handshake and time synchronization and
reports topics whose schema hash differs from the local build. Without
--check it shows a live view until interrupted: every topic's rate and latest
values, the Link statistics of both ends and every work item's timing (the
summary lines once a second when the output is not a terminal). With
--check it runs for a fixed time, prints a parseable summary and exits
non-zero when the handshake fails, a schema hash differs, frames are lost or
an expected rate is not met.

The param command lists, reads or sets parameters (spec 0005) instead,
describing them from the metadata file the parameter generator writes into
the build tree. It warns when that metadata does not match the MCU's
parameter set hash, and exits non-zero when the MCU refuses a request.

    link_monitor.py --udp 127.0.0.1:15600
    link_monitor.py --serial /dev/ttyACM0
    link_monitor.py --udp 127.0.0.1:15600 --check --duration 5 --expect-rate imu_sample:0=200
The mode, arm, disarm and estop commands change the mode, arm or disarm, and
engage or release the emergency stop (spec 0006), print the state now in
effect and exit non-zero when the MCU refuses. arm warns when arming checks
are skipped. sim-power cuts or restores SITL's simulated actuator power.
bringup motors runs the motor-to-encoder bring-up case in test mode while
armed, prints each output's result and what to change, and exits non-zero
unless every output passes.
drive requests TEST and arming if needed, sends one wheel speed per actuator
output (rad/s, nan leaves an output uncontrolled) at 20 Hz for the duration,
then all NaN, and disarms; it prints the wheel speeds reached and exits
non-zero when the vehicle disarms on the way.

    link_monitor.py --udp 127.0.0.1:15600 param list
    link_monitor.py --udp 127.0.0.1:15600 param set ATT_KP 2.5
    link_monitor.py --udp 127.0.0.1:15600 mode test
    link_monitor.py --udp 127.0.0.1:15600 arm
    link_monitor.py --udp 127.0.0.1:15600 estop on
    link_monitor.py --udp 127.0.0.1:15600 drive 10,10,10,10 --duration 3
    link_monitor.py --udp 127.0.0.1:15600 drive --duration 2 -- -5,nan,nan,nan
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from vklink.monitor import (  # noqa: E402
    LinkMonitor,
    speeds_text,
    ParamMetadata,
    RateExpectation,
    SerialConnection,
    UdpConnection,
    check,
    format_param,
    load_messages,
    load_param_metadata,
    parse_param_text,
)
from vklink.session import (  # noqa: E402
    BRINGUP_CASES,
    BRINGUP_RESULTS,
    DISARM_REASONS,
    MODES,
    BringupOutput,
    ControlReply,
    ParamReject,
    ParamType,
    ParamValue,
    check_names,
)
from vklink.view import render  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
REFRESH_S = 0.5
# ANSI: cursor home, clear the screen.
CLEAR = "\x1b[H\x1b[2J"
DEFAULT_MSG_DIR = REPO_ROOT / "build" / "sitl" / "generated" / "schema" / "python"
DEFAULT_PARAM_META = REPO_ROOT / "build" / "sitl" / "generated" / "params" / "params.json"
# Seconds to wait for the MCU to answer a parameter request.
PARAM_TIMEOUT_S = 3.0


def parse_address(text: str) -> tuple[str, int]:
    host, _, port = text.rpartition(":")
    if not host or not port.isdigit() or not 0 < int(port) < 65536:
        raise argparse.ArgumentTypeError(f"expected <address>:<port>, got '{text}'")
    return host, int(port)


def parse_expectation(text: str) -> RateExpectation:
    try:
        return RateExpectation.parse(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from None


def refused(name: str, reply: ParamReject | None) -> int:
    """Reports a request the MCU refused or never answered; the exit code."""
    if reply is None:
        print(f"FAIL: no reply from the MCU about {name}", file=sys.stderr)
    else:
        print(f"FAIL: {reply.name} rejected: {reply.reason_text}", file=sys.stderr)
    return 1


def parse_speeds(text: str) -> tuple[float, ...]:
    try:
        speeds = tuple(float(value) for value in text.split(","))
    except ValueError:
        speeds = ()
    if len(speeds) != 4:
        raise argparse.ArgumentTypeError(f"expected four wheel speeds <w1>,<w2>,<w3>,<w4> in rad/s, got '{text}'")
    return speeds


def parse_duration(text: str) -> float:
    try:
        duration = float(text)
    except ValueError:
        duration = -1.0
    if not duration > 0:
        raise argparse.ArgumentTypeError(f"expected a positive number of seconds, got '{text}'")
    return duration


def run_param(monitor: LinkMonitor, args: argparse.Namespace, metadata: ParamMetadata | None) -> int:
    """The param command, once the handshake is done."""
    if metadata is not None and metadata.set_hash != monitor.session.param_set_hash:
        print(f"warning: the MCU's parameter set hash 0x{monitor.session.param_set_hash:016X} differs from "
              f"0x{metadata.set_hash:016X} in {args.param_meta}; descriptions and ranges may be wrong",
              file=sys.stderr)
    meta = metadata.by_name if metadata is not None else {}

    if args.action == "list":
        params = monitor.list_params(PARAM_TIMEOUT_S)
        if params is None:
            print("FAIL: the parameter list did not arrive", file=sys.stderr)
            return 1
        for param in params:
            print(format_param(param, meta.get(param.name)))
        return 0

    if args.action == "set":
        if args.name in meta:
            kind = ParamType[meta[args.name]["type"]]
        else:
            # Not in the metadata: the MCU says what type it is.
            current = monitor.get_param(args.name, PARAM_TIMEOUT_S)
            if not isinstance(current, ParamValue):
                return refused(args.name, current)
            kind = ParamType(current.type)
        try:
            value = parse_param_text(args.value, kind, meta.get(args.name))
        except ValueError as error:
            print(f"error: {args.name}: {error}", file=sys.stderr)
            return 2
        reply = monitor.set_param(args.name, kind, value, PARAM_TIMEOUT_S)
    else:
        reply = monitor.get_param(args.name, PARAM_TIMEOUT_S)
    if not isinstance(reply, ParamValue):
        return refused(args.name, reply)
    print(format_param(reply, meta.get(reply.name)))
    return 0


# The motor case pulses four outputs for 0.6 s each.
BRINGUP_TIMEOUT_S = 10.0


def bringup_fix(output: int, item: BringupOutput) -> str:
    """What to change for an output that did not pass."""
    if item.result == 2:
        return (f"set MOT{output}_REV=REVERSED if the wheel turned backwards, "
                f"MOT{output}_ENC_REV=REVERSED if it turned forwards")
    if item.result == 3:
        return f"encoder {item.encoder} moved instead: check which encoder is connected to output {output}"
    return "check the motor, its power and its encoder"


def run_bringup(monitor: LinkMonitor, args: argparse.Namespace) -> int:
    """The bringup command, once the handshake is done: needs TEST and armed."""
    reply = monitor.run_bringup(BRINGUP_CASES[args.case], BRINGUP_TIMEOUT_S)
    if reply is None:
        print("FAIL: no bring-up result from the MCU", file=sys.stderr)
        return 1
    if reply.rejected:
        print(f"FAIL: bringup refused: {reply.reason_text}", file=sys.stderr)
        return 1
    for output, item in enumerate(reply.outputs, start=1):
        sign = {1: "+1", -1: "-1"}.get(item.sign, "0")
        line = (f"bringup output={output} result={BRINGUP_RESULTS.get(item.result, item.result).replace(' ', '_')} "
                f"encoder={item.encoder} sign={sign}")
        print(line if item.passed else f"{line}  fix: {bringup_fix(output, item)}")
    if reply.aborted:
        print("FAIL: the bring-up case was aborted", file=sys.stderr)
        return 1
    return 0 if all(item.passed for item in reply.outputs) else 1


def run_control(monitor: LinkMonitor, args: argparse.Namespace) -> int:
    """The mode, arm, disarm, estop and sim-power commands, once the handshake is done."""
    if args.command == "sim-power":
        monitor.sim_actuator_power(args.state == "on")
        print(f"sim_actuator_power={args.state}")
        return 0
    reply: ControlReply | None
    if args.command == "mode":
        mode = next(number for number, name in MODES.items() if name.lower() == args.mode)
        reply = monitor.request_mode(mode, PARAM_TIMEOUT_S)
    elif args.command in ("arm", "disarm"):
        reply = monitor.request_arming(args.command == "arm", PARAM_TIMEOUT_S)
    else:
        reply = monitor.request_estop(args.state == "on", PARAM_TIMEOUT_S)
    if reply is None:
        print(f"FAIL: no reply from the MCU to {args.command}", file=sys.stderr)
        return 1
    if reply.refused:
        print(f"FAIL: {args.command} refused: {reply.reason_text}", file=sys.stderr)
        return 1
    if args.command == "mode":
        print(f"mode={MODES.get(reply.value, reply.value)}")
    elif args.command in ("arm", "disarm"):
        skipped = check_names(reply.skipped)
        print(f"armed={reply.value} skipped_checks={','.join(skipped) or 'none'}")
        if args.command == "arm" and skipped:
            print(f"warning: arming checks skipped: {', '.join(skipped)}", file=sys.stderr)
    else:
        print(f"emergency_stop={reply.value}")
    return 0


def run_drive(monitor: LinkMonitor, args: argparse.Namespace) -> int:
    """The drive command, once the handshake is done."""
    test = next(number for number, name in MODES.items() if name == "TEST")
    if "actuator_setpoint" not in monitor.session.peer_topics_by_name:
        print("FAIL: the MCU does not offer actuator_setpoint", file=sys.stderr)
        return 1
    for command, request in (("mode", lambda: monitor.request_mode(test, PARAM_TIMEOUT_S)),
                             ("arm", lambda: monitor.request_arming(True, PARAM_TIMEOUT_S))):
        reply = request()
        if reply is None:
            print(f"FAIL: no reply from the MCU to {command}", file=sys.stderr)
            return 1
        if reply.refused:
            print(f"FAIL: {command} refused: {reply.reason_text}", file=sys.stderr)
            return 1
    skipped = check_names(reply.skipped)
    if skipped:
        print(f"warning: arming checks skipped: {', '.join(skipped)}", file=sys.stderr)

    try:
        disarmed = monitor.drive(args.speeds, args.drive_duration)
        encoders = monitor.latest("wheel_encoders")
    finally:
        # Disarm whatever happened, an interrupt included.
        disarm = monitor.request_arming(False, PARAM_TIMEOUT_S)
    speeds = speeds_text(encoders[0][1].speed) if encoders else "none"
    print(f"wheel_speed={speeds} armed={disarm.value if disarm is not None else 'unknown'}")
    if disarmed is not None:
        reason = DISARM_REASONS.get(disarmed.disarm_reason, disarmed.disarm_reason)
        print(f"FAIL: the vehicle disarmed while driving: {reason}", file=sys.stderr)
        return 1
    if disarm is None:
        print("FAIL: no reply from the MCU to disarm", file=sys.stderr)
        return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--udp", type=parse_address, default=("127.0.0.1", 15600), metavar="ADDRESS:PORT",
                        help="the Link's UDP address (default 127.0.0.1:15600)")
    target.add_argument("--serial", metavar="PATH", help="the Link's serial port, such as /dev/ttyACM0 or a pty")
    parser.add_argument("--baud", type=int, default=921600, help="serial baud rate (default 921600)")
    parser.add_argument("--msg-dir", default=os.environ.get("VK_MSG_PYTHON_DIR", DEFAULT_MSG_DIR),
                        help="directory of the generated vkmsg.py (default: the sitl build)")
    parser.add_argument("--handshake-timeout", type=float, default=5.0, help="seconds to wait for the MCU")
    parser.add_argument("--check", action="store_true", help="run for --duration, then judge the Link")
    parser.add_argument("--duration", type=float, default=5.0, help="seconds to run with --check")
    parser.add_argument("--expect-rate", type=parse_expectation, action="append", default=[],
                        metavar="TOPIC:INSTANCE=HZ", help="required rate of a topic instance, with --check")
    parser.add_argument("--tolerance", type=float, default=0.05, help="allowed relative rate error")
    parser.add_argument("--param-meta", default=os.environ.get("VK_PARAM_METADATA", DEFAULT_PARAM_META),
                        help="the parameter metadata file (default: the sitl build's)")
    commands = parser.add_subparsers(dest="command")
    param = commands.add_parser("param", help="list, read or set parameters")
    actions = param.add_subparsers(dest="action", required=True)
    actions.add_parser("list", help="every parameter with its value")
    get = actions.add_parser("get", help="one parameter's value")
    get.add_argument("name")
    set_ = actions.add_parser("set", help="change a parameter; enum values may be given by name")
    set_.add_argument("name")
    set_.add_argument("value")
    mode = commands.add_parser("mode", help="change the mode; only while disarmed")
    mode.add_argument("mode", choices=[name.lower() for name in MODES.values()])
    commands.add_parser("arm", help="arm, if every arming check passes")
    commands.add_parser("disarm", help="disarm")
    estop = commands.add_parser("estop", help="engage or release the emergency stop")
    estop.add_argument("state", choices=["on", "off"])
    bringup = commands.add_parser("bringup", help="run a bring-up case; needs TEST and armed")
    bringup.add_argument("case", choices=list(BRINGUP_CASES))
    sim_power = commands.add_parser("sim-power", help="SITL only: cut or restore the simulated actuator power")
    sim_power.add_argument("state", choices=["on", "off"])
    drive = commands.add_parser("drive", help="arm and drive the wheels for a while, then disarm")
    drive.add_argument("speeds", type=parse_speeds, metavar="W1,W2,W3,W4",
                       help="wheel speeds in rad/s by output number, nan for none; put -- before a negative first one")
    drive.add_argument("--duration", dest="drive_duration", type=parse_duration, required=True, metavar="SECONDS",
                       help="seconds to send the setpoints")
    args = parser.parse_args()

    metadata = None
    if args.command == "param":
        try:
            metadata = load_param_metadata(args.param_meta)
        except FileNotFoundError:
            print(f"warning: no parameter metadata at {args.param_meta}; showing names and values only",
                  file=sys.stderr)

    try:
        messages = load_messages(args.msg_dir)
    except FileNotFoundError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    try:
        if args.serial:
            connection = SerialConnection(args.serial, args.baud)
            target = f"serial {args.serial} {args.baud} baud"
        else:
            connection = UdpConnection(*args.udp)
            target = "udp {}:{}".format(*args.udp)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    monitor = LinkMonitor(connection, messages)
    try:
        if not monitor.handshake(args.handshake_timeout):
            print("\n".join(monitor.summary()))
            print(f"FAIL: no handshake with the MCU within {args.handshake_timeout:g} s", file=sys.stderr)
            return 1
        if args.command == "param":
            return run_param(monitor, args, metadata)
        if args.command == "drive":
            return run_drive(monitor, args)
        if args.command == "bringup":
            return run_bringup(monitor, args)
        if args.command is not None:
            return run_control(monitor, args)
        if args.check:
            monitor.run(args.duration)
            print("\n".join(monitor.summary()))
            result = check(monitor, args.expect_rate, args.tolerance)
            for failure in result.failures:
                print(f"FAIL: {failure}", file=sys.stderr)
            return 0 if result.passed else 1
        live = sys.stdout.isatty()
        if not live:
            for mismatch in monitor.mismatches:
                print(f"warning: topic {mismatch.name} has a different schema hash on the MCU", file=sys.stderr)
        while True:
            monitor.run(REFRESH_S if live else 1.0)
            if live:
                width = shutil.get_terminal_size().columns
                print(CLEAR + "\n".join(render(monitor, target, width)), flush=True)
            else:
                print("\n".join(monitor.summary()), flush=True)
    except KeyboardInterrupt:
        return 0
    finally:
        connection.close()


if __name__ == "__main__":
    sys.exit(main())
