# Contributing to Vehkernel

Thank you for helping. This page covers how work is proposed, how it is built and checked, and what a pull request needs before it is merged. Everyone taking part follows the [Code of Conduct](CODE_OF_CONDUCT.md).

## Before you start

- **Read the design first.** The [README](README.md) gives the overview, the [ADRs](docs/adr/README.md) record why the firmware is built the way it is, and [`GLOSSARY.md`](GLOSSARY.md) defines the vocabulary. The most important boundary is [ADR-0001](docs/adr/0001-mcu-responsibility-boundary.md): the MCU does real-time estimation, low-level control and safety, never global localization or navigation.
- **Open or find an issue before large changes.** A new board, driver, module or Link protocol, or anything that changes an ADR, starts as an issue so the approach can be agreed before code is written. Small fixes and documentation changes can go straight to a pull request.
- **Write in English** with the glossary's terms, in the style of ArduPilot and PX4 documentation ([ADR-0021](docs/adr/0021-english-project-language.md)).

## Development environment

Everything that affects build output is pinned in the development container's image, and `cmake` and `ctest` run inside it, never on the host ([ADR-0016](docs/adr/0016-dependency-policy.md)). Clone with submodules, then open the repository in VS Code Dev Containers or run each command through Docker, as [Building](docs/guide/building.md#development-environment) shows:

```bash
git clone --recurse-submodules https://github.com/chimu0510/Vehkernel.git
```

[Building](docs/guide/building.md) lists the presets and the host tools needed for flashing. You do not need hardware to contribute: SITL runs the complete firmware on the development host ([SITL](docs/guide/sitl.md)).

## Checking your change

Run what CI runs before you open a pull request, in the container (with Docker, prefix each command with `vk`):

```bash
cmake --preset test
cmake --build --preset test
ctest --preset test
```

and build every firmware preset you touched, for example `c50c-v1_1`. A change to the Link's decoders also runs the fuzz targets (`cmake --preset fuzz`, `cmake --build --preset fuzz`, `ctest --preset fuzz`). The `sitl`, `test` and `fuzz` presets run clang-tidy with every warning as an error. [Testing](docs/guide/testing.md) describes the six test layers; add tests at the lowest layer that can catch the bug. Changes that move actuators also need the hardware check described there, done with the wheels off the ground and output power limited.

## Coding rules

- C++20 subset: no exceptions, no RTTI, no dynamic allocation after initialization, no allocating STL containers ([ADR-0007](docs/adr/0007-cpp20-subset.md)).
- Modules talk only through the bus; Links never drive actuators directly, every setpoint goes through setpoint arbitration ([ADR-0002](docs/adr/0002-bus-backbone-and-pluggable-links.md), [ADR-0005](docs/adr/0005-setpoint-arbitration.md)).
- Topics are defined in `schema/msg/` and code is generated from them; never hand-edit generated files ([ADR-0006](docs/adr/0006-schema-and-wire-encoding.md)).
- Hardware facts belong in the board definition, runtime choices in parameters, build-time choices in Kconfig ([ADR-0011](docs/adr/0011-toml-board-definition-toolchain.md), [ADR-0012](docs/adr/0012-parameter-system.md), [ADR-0017](docs/adr/0017-kconfig-build-configuration.md)).
- Third-party code is pinned and declared centrally ([ADR-0016](docs/adr/0016-dependency-policy.md)).
- Match the surrounding code: its naming, its comment density, and a reference to the ADR a file implements at the top of the file. [`.editorconfig`](.editorconfig) sets whitespace.
- New files start with an `SPDX-License-Identifier` line for `GPL-3.0-or-later` in their own comment syntax (after a shebang), with no copyright line; files that cannot hold a comment are listed in [`REUSE.toml`](REUSE.toml), and `reuse lint` checks both.
- Do not commit vendor datasheets, schematics, firmware or flash dumps: they are not ours to redistribute. Keep them under the ignored `vendor/` directory.

## Decisions and milestones

- A decision that is hard to reverse, surprising without its context and the result of a real trade-off gets an ADR. Changing an existing decision means editing that ADR in place, with the reason in the commit message ([`docs/adr/README.md`](docs/adr/README.md)).
- Every new board gets its own board definition, ADR and spec ([ADR-0010](docs/adr/0010-board-revision-policy.md)).
- A milestone is a spec: a GitHub issue titled `Spec: ...` plus a file in `docs/specs/`, with tickets as sub-issues ([`docs/specs/README.md`](docs/specs/README.md)).

## Issues

Use the issue forms for bug reports and feature proposals. Maintainers triage new issues with these labels:

| Label | Meaning |
|---|---|
| `needs-triage` | A maintainer has not evaluated it yet |
| `needs-info` | Waiting on the reporter for more information |
| `ready-for-agent` | Fully specified; anyone, including an AI coding agent, can pick it up |
| `ready-for-human` | Needs a person, usually because it needs hardware on the bench |
| `wontfix` | Will not be actioned |

To work on a ticket, comment on it or assign it to yourself so nobody else starts the same work. Progress and results go in the issue comments ([handoff](docs/agents/handoff.md)).

## Commits and pull requests

- Commit subjects start with the area they touch, as in the history: `modules: ...`, `platform: ...`, `drivers: ...`, `tools: ...`, `docs: ...`, `build: ...`. The body says why, not only what.
- One logical change per commit; a pull request may hold several.
- Link the issue the pull request resolves (`Closes #NN`), and fill in the pull request template.
- Update the docs, the ADRs and the spec your change affects in the same pull request.
- AI-assisted contributions are welcome. You are responsible for every line you submit; [`AGENTS.md`](AGENTS.md) holds the rules agents follow in this repository.

## License

Vehkernel is licensed under GPL-3.0-or-later. By contributing, you agree that your contribution is licensed under the same terms ([LICENSE](LICENSE)).
