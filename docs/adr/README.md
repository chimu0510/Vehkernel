# Architecture decision records

An ADR records a decision that is hard to reverse, surprising without its context and the result of a real trade-off. Every board also gets one when it is added (ADR-0010). Smaller choices live in the code, the specs and the tickets.

## Format

- File name `NNNN-slug.md`, numbered after the highest existing ADR. Cite ADRs as `ADR-NNNN`.
- The title states the decision, not the topic.
- The body opens with the context and the decision. Sub-topics of a long decision start with a bold lead line, such as `**Write policy**`.
- `## Considered Options` lists the rejected or deferred alternatives worth remembering, each as **name**: verdict, then why.
- `## Consequences` lists downstream effects a reader would not expect.
- Both sections are optional; leave out a section with nothing worth saying.

## Editing

ADRs are living records with no status field: each one always reads as the current decision.

- When a later decision changes an earlier ADR, edit that ADR in place; git history keeps the old text. The commit message says why.
- Implementing a decision does not shorten its ADR: keep the discussion, the considered options and what implementation showed. Remove only text that a later decision rejected or changed.
- A decision changed while a spec is implemented is written into every ADR it touches, as well as into the spec (`../specs/README.md`).

## Index

| ADR | Decision |
|---|---|
| [0001](0001-mcu-responsibility-boundary.md) | The MCU does real-time estimation and low-level control, not navigation |
| [0002](0002-bus-backbone-and-pluggable-links.md) | The internal bus is the backbone; ROS2 is just one Link |
| [0003](0003-vehicle-level-ros2-api.md) | Vehicle-level topic API for ROS2; ros2_control is an optional adapter |
| [0004](0004-freertos.md) | FreeRTOS as the RTOS |
| [0005](0005-setpoint-arbitration.md) | Setpoint arbitration: the mode decides who is in control, and RC can always take over |
| [0006](0006-schema-and-wire-encoding.md) | `.msg`-style schema; the custom protocol uses packed little-endian encoding |
| [0007](0007-cpp20-subset.md) | The firmware is written in a subset of C++20 |
| [0008](0008-bus-and-work-queues.md) | Bus semantics and a rate-tiered Work Queue execution model |
| [0009](0009-layered-hal.md) | Layered HAL; the form of polymorphism follows when the choice is made |
| [0010](0010-board-revision-policy.md) | Boards are added one at a time with their own ADR and spec, verify their revision at boot, and allow alt configs per pin |
| [0011](0011-toml-board-definition-toolchain.md) | TOML board definitions with an SoC database and a DMA solver |
| [0012](0012-parameter-system.md) | Parameter system: direct struct access, per-entry storage keyed by name hash |
| [0013](0013-control-chain-and-vehicle-models.md) | One shared control chain; frame differences live only in the vehicle model |
| [0014](0014-estimation-architecture.md) | Estimation split into replaceable estimator modules and a math component library |
| [0015](0015-selectable-timing.md) | Timing as selectable dimensions, not a fixed scheme |
| [0016](0016-dependency-policy.md) | Third-party tools and packages are allowed, pinned and declared centrally |
| [0017](0017-kconfig-build-configuration.md) | Build-time configuration is solved by Kconfig; CMake only builds |
| [0018](0018-test-layers-and-sitl.md) | Six test layers; SITL runs the same RTOS |
| [0019](0019-logging.md) | ULog logging with replaceable sinks; backpressure handled in the logger |
| [0020](0020-internal-flash-layout-and-write-policy.md) | Internal Flash layout and write policy: write but never erase while armed |
| [0021](0021-english-project-language.md) | English is the project language |
| [0022](0022-local-display-and-local-menu.md) | Local display and local menu: status always, whitelisted parameters only while disarmed |
| [0023](0023-bootloader.md) | Bootloader: the ArduPilot and PX4 protocol, kept where a board has it, ported where it does not |
| [0024](0024-c50c-v1_1-board.md) | The C50C V1.1: an STM32F407 robot controller with the Vehkernel bootloader on its two serial Links |
| [0026](0026-host-tools-bundle.md) | The host tools reach a companion computer as a bundle built with the firmware |
