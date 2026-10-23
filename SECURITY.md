# Security and safety policy

Vehkernel drives actuators, so a flaw can hurt people or damage hardware as well as leak data. Treat both kinds of issue as security issues.

## Supported versions

Vehkernel has no releases yet. Only the `main` branch is supported; fixes land there.

## What to report privately

- A way for a Link, a setpoint source or a malformed frame to move actuators while disarmed, bypass setpoint arbitration, the emergency stop, arming checks or the output watchdog ([ADR-0005](docs/adr/0005-setpoint-arbitration.md)).
- Memory safety bugs in a Link decoder, the parameter system or the bootloader, such as a crafted frame that corrupts memory or crashes the firmware.
- A way to write Flash or flash firmware that bypasses the documented paths ([ADR-0020](docs/adr/0020-internal-flash-layout-and-write-policy.md), [ADR-0023](docs/adr/0023-bootloader.md)).
- Secrets or third-party material that should not be in the repository.

Ordinary bugs, including control or estimation bugs that need a deliberate configuration error to trigger, go in a public [bug report](https://github.com/chimu0510/Vehkernel/issues/new/choose).

## How to report

Use GitHub's [private vulnerability reporting](https://github.com/chimu0510/Vehkernel/security/advisories/new). Include the commit, the board or SITL, the steps or input that trigger it, and what happens. Please do not disclose the issue publicly until a fix is on `main`.

You will get an acknowledgement within 7 days. Once the issue is confirmed, the fix is developed in a private advisory and the advisory is published with the fix, crediting you unless you prefer otherwise.
