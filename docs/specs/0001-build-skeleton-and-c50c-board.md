# Spec 0001: build skeleton and C50C V1.1 board definition

**Status**: done. Issue: [#4](https://github.com/chimu0510/Vehkernel/issues/4); tickets #5 to #13. The boot path was confirmed on hardware in [#10](https://github.com/chimu0510/Vehkernel/issues/10).

This is the spec as written before implementation. Later decisions live in the ADRs and the code; where they differ, they win.

## Problem Statement

Vehkernel has architecture decisions (ADR-0001 to ADR-0020) and a glossary, but nothing that builds. Firmware developers cannot start on the bus, drivers or estimators because:

- There is no pinned development environment, so toolchain versions differ between machines and results are not reproducible.
- There is no build system and no Kconfig flow to decide what the firmware contains.
- There is no board definition for C50C V1.1 and no MCU database for the STM32F407VET6; pins, DMA and the Flash layout exist only in schematics and ADR prose.
- There is no generator, so the promises of the ADRs (pin misuse failing at build time, automatic DMA solving, alt config conflict checks) cannot be verified.
- There is no CI, so nothing confirms that a change keeps the build working or stays within the Flash budget.

## Solution

Deliver the first milestone: **build skeleton and the C50C V1.1 board definition**. Afterwards a developer opens the devcontainer, picks a CMake preset (`c50c-v1_1`, `sitl` or `test`), and configures and builds:

- The board generator reads the C50C V1.1 board definition and the STM32F407VET6 MCU database, validates every pin, solves DMA automatically, checks every alt config, and emits typed pin constants, Kconfig symbols, linker script memory regions and board default parameters. A wrong board definition fails at configure time and names the line and the pin.
- Kconfig merges MCU defaults → board defaults → debug overlay, validates the constraints, and generates a configuration header.
- `c50c-v1_1` produces a minimal firmware image that can be flashed and boots: a jump stub in S0 hands control to the firmware starting at S4, following the Flash layout of ADR-0020.
- Every build checks Flash, RAM and CCM usage and fails when a board budget is exceeded.
- On every push, CI runs the three presets, the L0 static checks and the generator tests in the same devcontainer.

## User Stories

1. As a firmware developer, I want to open the repo in a devcontainer with every tool installed, so that I can start building without hunting for toolchain versions.
2. As a firmware developer, I want the cross compiler, CMake, Ninja and Python pinned to exact versions, so that my build matches CI and every teammate's build.
3. As a firmware developer, I want Python dependencies resolved from a lock file, so that the generators behave the same on every machine.
4. As a firmware developer, I want to build without network access once the devcontainer is ready, so that builds are reproducible and work offline.
5. As a firmware developer, I want one CMake preset per build target (`c50c-v1_1`, `sitl`, `test`), so that I select a target by name instead of passing flags.
6. As a firmware developer, I want the `c50c-v1_1` preset to produce a firmware image I can flash over USB DFU or SWD, so that I can verify the toolchain end to end on real hardware.
7. As a firmware developer, I want the firmware to start at sector S4 with the vector table relocated there, so that S0 stays free for a future bootloader as ADR-0020 decides.
8. As a firmware developer, I want a jump stub in S0 that hands control to the firmware in S4, so that the board boots without a bootloader.
9. As a firmware developer, I want the `sitl` preset to configure and build a host executable, so that the SITL path is wired into the build from day one.
10. As a firmware developer, I want the `test` preset to build and run the host test suites, so that I have one command for local testing.
11. As a firmware developer, I want Kconfig to decide which features are compiled in, so that `depends on`, `select` and `choice` constraints are solved by a real solver instead of ad-hoc CMake logic.
12. As a firmware developer, I want Kconfig to merge MCU defaults, board defaults and a debug overlay in that order, so that each layer can override the one below it.
13. As a firmware developer, I want Kconfig to re-validate every constraint after each merge, so that an overlay cannot silently produce an invalid configuration.
14. As a firmware developer, I want a debug overlay I can apply locally without editing board defaults, so that debug options never leak into a board's committed configuration.
15. As a firmware developer, I want the solved configuration available to C++ as a generated header, so that code can test configuration symbols at compile time.
16. As a firmware developer, I want an interactive way to inspect and change a preset's Kconfig configuration, so that I can explore options without reading every Kconfig file.
17. As a board definition author, I want each pin to be one line of TOML with its role and function, so that I can compare the definition with the schematic line by line.
18. As a board definition author, I want the generator to reject a function that does not exist on a pin according to the MCU database, so that impossible assignments fail at configure time.
19. As a board definition author, I want the generator to derive the AF number from the function name, so that I never write AF numbers by hand.
20. As a board definition author, I want the generator to reject two roles on the same pin, so that pin conflicts are caught before the code compiles.
21. As a board definition author, I want the generator to reject two pins claiming the same timer channel or peripheral signal, so that timer conflicts are caught early.
22. As a board definition author, I want error messages to name the file, line and pin, so that I can fix the definition quickly.
23. As a board definition author, I want the generator to solve DMA stream and channel assignments, so that I never fill DMA tables by hand.
24. As a board definition author, I want the DMA solution to be deterministic, so that the same definition always yields the same assignment and diffs stay clean.
25. As a board definition author, I want a DMA assignment report, so that I can review which request got which stream and why.
26. As a board definition author, I want to annotate a request with a priority, "no DMA" or "sharing allowed", so that I can steer the solver when the hardware forces a trade-off.
27. As a board definition author, I want an unsolvable DMA configuration to fail with the conflicting requests named, so that I know which annotation or pin to change.
28. As a board definition author, I want to declare alt configs through a pin's `alt` field, so that a pin can serve another purpose under another frame.
29. As a board definition author, I want the generator to check every alt config combination for pin and timer conflicts, so that no frame selection at boot can produce a conflict.
30. As a board definition author, I want to declare logical serial ports such as `SERIAL1` instead of fixed uses, so that each port's use is assigned at runtime by `SERIALn_FUNCTION`.
31. As a board definition author, I want to give each logical serial port a default use, so that the board works out of the box.
32. As a board definition author, I want to declare board devices with their mounting orientation and peripheral bus type, so that estimators can rotate sensor data and the timing budget knows the bus cost.
33. As a board definition author, I want to declare the board's storage devices, so that the right log sinks are compiled in.
34. As a board definition author, I want to declare the internal Flash layout of ADR-0020 in the board definition, so that the linker script and Flash budget follow from one source.
35. As a board definition author, I want the generator to check the Flash layout against the MCU's sector table, so that no region starts or ends inside a sector.
36. As a board definition author, I want to declare board default parameters, including the timing preset, so that a board's defaults live next to its hardware facts.
37. As a board definition author, I want board definitions to support `extends` and `remove`, so that a future board revision can reuse a base definition.
38. As a board definition author, I want the generator to export hardware capabilities as Kconfig symbols such as `BOARD_HAS_CAN` and `BOARD_IMU_BUS_SOFT_I2C`, so that drivers depend on hardware facts without duplicating them.
39. As a board definition author, I want a JSON dump of the resolved board definition, so that tools and reviewers can inspect the final result.
40. As a firmware developer, I want typed pin constants generated for the board, so that using a pin in the wrong role fails to compile.
41. As a firmware developer, I want the generated clock configuration and peripheral instance table, so that platform code never hard-codes board facts.
42. As a firmware developer, I want each board's Flash, RAM and CCM budgets enforced on every build, so that outgrowing the hardware fails immediately.
43. As a firmware developer, I want a budget failure to report how far each region is over, so that I know how much to cut.
44. As a firmware developer, I want generated code kept out of version control, so that it can never drift from its sources.
45. As a C50C V1.1 user, I want a complete C50C V1.1 board definition matching the schematic, the vendor firmware and bench measurements, so that the firmware drives the real board correctly.
46. As a C50C V1.1 user, I want the revision-ID pins PE2, PE3 and PC13 declared in the board definition, so that the boot-time revision check of ADR-0010 can be implemented later.
47. As a C50C V1.1 user, I want the IMU declared on software I2C, the provisional default of ADR-0015, so that the board builds now and can switch to hardware I2C once bring-up confirms the wiring.
48. As a C50C V1.1 user, I want encoder C on PB7 and its servo alt config declared, so that the ACKERMANN configuration of ADR-0010 is representable.
49. As a board definition author, I want the STM32F407VET6 MCU database committed as JSON with its data source recorded, so that builds never need the network and the data's provenance can be audited.
50. As a board definition author, I want an offline conversion tool that regenerates the MCU database from upstream STM32 data, so that adding another MCU follows the same path.
51. As a board definition author, I want the MCU database to hold the AF table, the DMA request map and the Flash sector table, so that every generator check has a single data source.
52. As a maintainer, I want CI to run on every push in the same devcontainer, so that CI failures reproduce locally.
53. As a maintainer, I want CI to build all three presets, so that no target breaks silently.
54. As a maintainer, I want CI to run the L0 static checks (`-Werror`, clang-tidy, generator determinism, board definition validation), so that regressions are caught before review.
55. As a maintainer, I want CI to run the generator twice and compare the outputs, so that nondeterminism is caught as soon as it appears.
56. As a maintainer, I want CI to run the generator test suite, so that every validation rule stays covered.
57. As a maintainer, I want CI to fail when a board exceeds its size budget, so that budgets are enforced, not advisory.
58. As a new contributor, I want the README to explain how to open the devcontainer and build each preset, so that my first build takes minutes, not hours.
59. As a new contributor, I want generator errors to use the vocabulary of GLOSSARY.md, so that messages match the documentation.
60. As a porter, I want a new board to need only a new board definition and board defaults, so that porting never touches the build system.
61. As a porter, I want a new STM32 MCU to need only a new MCU database and MCU defaults, so that the generator stays MCU-agnostic within the STM32 family.
62. As an integrator, I want every preset to build from a clean clone with one configure and one build command, so that automation scripts stay trivial.

## Implementation Decisions

- **Development environment**: a devcontainer pins the cross compiler, CMake, Ninja, Python and analysis tools to exact versions; Python packages come from a lock file (ADR-0016). kconfiglib is unmaintained and may be kept as a copy in the repo.
- **Build system**: CMake + `CMakePresets.json` + Ninja, with three presets: `c50c-v1_1` (cross build), `sitl` (development host executable) and `test` (development host tests). Generators run as CMake custom commands and write to the build directory; their output is never committed (ADR-0017).
- **Kconfig**: solved with kconfiglib during CMake configuration. Three layers, MCU defaults → board defaults → debug overlay, re-validated after each merge. `select` only for helper libraries without dependencies of their own, `depends on` everywhere else. Kconfig decides what is compiled in and holds no runtime choices (ADR-0017). This milestone needs only a few demonstration symbols to prove the flow.
- **MCU database**: one for the STM32F407VET6, with the AF table, DMA request map and Flash sector table, stored as JSON with its data source recorded. A conversion tool runs offline on the STM32Cube database (possibly through the stm32-data project); builds read only the JSON (ADR-0011, ADR-0016).
- **Board generator**: a standalone command-line program that takes a board definition and the MCU database and produces every output or a set of diagnostics. It:
  - Parses TOML and resolves `extends` / `remove`.
  - Validates pins and functions, derives AF numbers, and detects pin and timer channel conflicts.
  - Solves DMA by backtracking, deterministically, with an assignment report and the priority, no-DMA and sharing annotations.
  - Expands and checks every alt config combination.
  - Handles logical ports, board devices (mounting orientation, peripheral bus type), storage devices and the Flash layout (aligned to the MCU sector table).
  - Emits typed pin constants, clock configuration, DMA assignments, the peripheral instance table, board default parameters (including the timing preset), Kconfig symbols such as `BOARD_HAS_*`, linker script memory regions with the Flash budget, and a JSON dump.
  - Reports diagnostics with file, line and pin, in GLOSSARY.md vocabulary.
  - Produces byte-identical output for identical input.
- **C50C V1.1 board definition**: written from the schematic, vendor firmware and bench measurements rather than the vendor resource allocation table (ADR-0010). It covers the revision-ID pins PE2, PE3 and PC13; the IMU (ICM20948) provisionally on software I2C (ADR-0015, pending bring-up); encoder C on PB7 with its servo alt config; the ADR-0020 Flash layout; and the logical ports with default uses.
- **Flash layout and boot**: S0 holds the jump stub, S1–S2 are reserved for parameters, S3 for the event log, and S4–S7 hold the firmware (448 KB), with VTOR pointing at S4 (ADR-0020). This milestone only boots into an idle loop and does not start FreeRTOS.
- **Size budgets**: every build checks Flash, RAM and CCM usage. The Flash budget is the firmware region of the Flash layout; RAM and CCM budgets come from the MCU database and the board definition. Exceeding a budget fails the build and reports the overage per region (ADR-0017).
- **Layering**: CMake target dependencies follow the layering of ADR-0009; this milestone only creates empty targets for the layers so later code is constrained from the start.
- **CI**: GitHub Actions running inside the devcontainer image. On every push: build all three presets with their size budgets, run the L0 static checks and the generator tests (ADR-0018).

## Testing Decisions

- **Good tests check external behaviour only**: give the generator an input and check its outputs or diagnostics, never internal functions such as the DMA solver's intermediate state. Refactoring the generator's internals should not require changing tests.
- **Primary seam: the board generator's command line.** Tests use fixed inputs (small board definitions and a trimmed MCU database) with expected outputs (golden files). They cover:
  - Correct pin constants, AF numbers, Kconfig symbols, linker script regions and board default parameters for valid definitions.
  - One failing input per validation rule, checking that the diagnostic names the right line and pin: unknown function, duplicate pin, timer channel conflict, alt config conflict, unsolvable DMA, Flash region not aligned to sectors.
  - DMA annotations changing the result; two runs on the same input producing byte-identical output.
  - Inheritance through `extends` / `remove`.
  - The real C50C V1.1 board definition as a golden test that locks its outputs.
- **Secondary seam: CMake preset smoke tests in CI.** All three presets configure and build from a clean state; an artificially enlarged firmware fails the budget check; a Kconfig overlay that violates a constraint fails configuration.
- **L0 static checks**: `-Werror`, clang-tidy, generator determinism (run twice and compare) and board definition validation, all in CI.
- **Hardware confirmation**: the `c50c-v1_1` image, flashed over USB DFU or SWD, boots into the idle loop; a developer confirms this by hand, as it is not yet part of L4.
- **Prior art**: the repo has no tests yet; this milestone creates the first. The generator test framework is pinned per ADR-0016; on the C++ side GoogleTest only needs to be wired up for later L1 tests.

## Out of Scope

- The `core/` implementation: bus, work queues, parameter runtime and timing monitor (ADR-0008, ADR-0012, ADR-0015).
- FreeRTOS integration, both the STM32 and the POSIX ports; the `sitl` preset builds only an empty development-host executable for now.
- Device drivers, estimators, the control chain, vehicle models, Links and logging modules.
- The schema and parameter generators (ADR-0006, ADR-0012); board default parameters are emitted only as data, with no parameter system to read them yet.
- The boot-time board revision check (ADR-0010), bring-up test cases and the L4 hardware test runner (ADR-0018).
- A bootloader for firmware updates over a Link; S0 holds only the jump stub (ADR-0020).
- Boards other than the C50C and MCUs other than the STM32F407VET6.
- The ROS2 Bridge and L5 integration tests.

## Further Notes

- The C50C V1.1 IMU wiring is unverified: the vendor firmware says SCL = PB9 and SDA = PB8, the reverse of F407 hardware I2C1 (ADR-0015). The board definition starts on software I2C; if bring-up shows the wiring matches hardware I2C1, only the board definition and board defaults change.
- The vendor resource allocation table lists PC13, PE2 and PE3 as unused, although they are the revision-ID pins, and its OLED pins do not reflect the V1.2 change (ADR-0010).
- The generator catches only impossible configurations, not legal but wrong ones such as swapped encoder A/B channels, so the schematic must still be checked by hand (ADR-0011).
- After this milestone, the implementation details duplicated in ADR-0011, ADR-0017 and ADR-0020 are trimmed back to the decisions and their reasons, per docs/agents/domain.md.

