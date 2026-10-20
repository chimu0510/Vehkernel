"""The Link monitor's live view: a terminal table refreshed in place (spec 0003)."""

from __future__ import annotations

from .monitor import (
    HANDSHAKE_STATES,
    LinkMonitor,
    actuator_outputs_line,
    euler_degrees,
    vehicle_status_line,
    wheel_encoders_line,
    work_item_name,
)

# Topics shown in their own lines or tables rather than by their raw values.
STATUS_TOPICS = ("link_status", "work_item_status", "parameter_status", "vehicle_status", "wheel_encoders",
                 "actuator_outputs")


def format_value(value: object) -> str:
    if isinstance(value, float):
        return f"{value:.3f}"
    if isinstance(value, (tuple, list)):
        return "(" + ", ".join(format_value(v) for v in value) + ")"
    return str(value)


def format_message(message: object | None) -> str:
    if message is None:
        return "-"
    parts = []
    for name in getattr(message, "FIELDS", ()):
        if name.startswith("timestamp"):
            continue
        if name == "q":
            # An attitude reads better as angles than as a quaternion.
            roll, pitch, yaw = euler_degrees(message.q)
            parts.append(f"roll={roll:.1f}° pitch={pitch:.1f}° yaw={yaw:.1f}°")
        else:
            parts.append(f"{name}={format_value(getattr(message, name))}")
    return " ".join(parts)


def render(monitor: LinkMonitor, target: str, width: int = 120) -> list[str]:
    """The live view as lines, each cut to `width` columns."""
    stats = monitor.reader.stats
    state = f"handshake ok, protocol {monitor.session.version}" if monitor.connected else "no handshake"
    header = f"Vehkernel Link  {target}  {state}"
    if monitor.time_sync is not None:
        header += f"  rtt {monitor.time_sync.rtt_us / 1000:.1f} ms"
    lines = [
        header,
        f"host received  frames {stats.frames}  crc_errors {monitor.crc_errors}  "
        f"malformed {stats.malformed}  lost {monitor.lost}",
    ]
    for instance, status in monitor.latest("link_status"):
        lines.append(
            f"mcu link {instance}     sent {status.frames_sent} frames / {status.bytes_sent} B  "
            f"received {status.frames_received} frames / {status.bytes_received} B  drops {status.drops}  "
            f"crc_errors {status.crc_errors}  malformed {status.malformed}  lost {status.lost}  "
            f"{HANDSHAKE_STATES.get(status.handshake_state, '?')}"
        )
    for _, status in monitor.latest("parameter_status"):
        lines.append(
            f"parameters     {status.parameter_count}  sector {status.active_sector} "
            f"{status.bytes_used}/{status.sector_size} B  unsaved {status.unsaved}  "
            f"pending reboot {status.pending_reboot}  skipped at boot "
            f"{status.skipped_unknown + status.skipped_wrong_type + status.skipped_out_of_range + status.skipped_corrupt}"
            f"{'  FLASH FAILED' if status.failed else ''}"
        )
    for _, status in monitor.latest("vehicle_status"):
        lines.append("vehicle        " + vehicle_status_line(status).removeprefix("vehicle_status "))
    for _, encoders in monitor.latest("wheel_encoders"):
        lines.append("wheels         " + wheel_encoders_line(encoders).removeprefix("wheel_encoders "))
    for _, outputs in monitor.latest("actuator_outputs"):
        lines.append("outputs        " + actuator_outputs_line(outputs).removeprefix("actuator_outputs "))
    for mismatch in monitor.mismatches:
        lines.append(f"MISMATCH       topic {mismatch.name}: schema hash differs from the local build")

    lines += ["", f"{'Topic':<20} {'Inst':>4} {'Rate Hz':>9} {'Messages':>9}  Latest"]
    for (name, instance), topic in sorted(monitor.topics.items()):
        latest = "" if name in STATUS_TOPICS else format_message(topic.latest)
        lines.append(f"{name:<20} {instance:>4} {topic.rate_hz:>9.2f} {topic.messages:>9}  {latest}")

    work_items = monitor.latest("work_item_status")
    if work_items:
        lines += ["", f"{'Work item':<20} {'Runs':>9} {'Overruns':>9} {'Last us':>8} {'Mean us':>8} "
                      f"{'Max us':>8} {'Jitter us':>10}"]
        for _, status in work_items:
            lines.append(
                f"{work_item_name(status):<20} {status.runs:>9} {status.overruns:>9} {status.run_time_last_us:>8} "
                f"{status.run_time_mean_us:>8} {status.run_time_max_us:>8} {status.jitter_max_us:>10}"
            )
    return [line[:width] for line in lines]
