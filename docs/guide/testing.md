# Testing

Tests are split into six layers, L0 to L5 (ADR-0018). The directories under `test/` follow them: `unit/` and `fuzz/` (L1), `replay/` (L2), `sitl/` (L3), `hardware/` (L4) and `ros2/` (L5); L0 static checks run from the root config files and CI.

## Host tests and SITL tests

Unit tests and SITL tests run through `ctest` in the devcontainer:

```bash
cmake --preset test
cmake --build --preset test
ctest --preset test
```

The SITL tests (`sitl_*`) start the SITL executable and check it from the development host with the scripts in `test/sitl/`; each guide names the tests that cover it ([SITL](sitl.md), [Link monitor](link-monitor.md), [Driving the wheels](driving.md), [Local display](local-display.md)). SITL tests run serially under `ctest -j`, since they judge timing on the shared host. CI builds every preset and runs the tests inside the devcontainer image.

## Fuzzing the Link decoders

The Link decodes bytes from any connected host, so its decoders are fuzzed with libFuzzer (L1, ADR-0018). The `fuzz` preset builds three fuzz targets with Clang, AddressSanitizer, UndefinedBehaviorSanitizer and libstdc++'s bounds checks; the last catch an overrun from one array member into the next, which AddressSanitizer cannot see. The targets live in `test/fuzz/`:

| Target | Input | Checks besides the sanitizers |
|---|---|---|
| `vk_fuzz_cobs` | The bytes between two delimiters, through the COBS decoder | Decoding never overruns its output; anything decoded, encoded again, decodes the same; any input round-trips through the encoder |
| `vk_fuzz_frame_reader` | A byte stream through the frame reader: COBS, length, CRC, frame type and sequence | Frames passed on have a known type and fit; they are read back the same when written again; each delimiter ends at most one frame |
| `vk_fuzz_session` | Frames of any type and body, in any order and at any time step, through `Session::handle()` with the build's parameters and a fake vehicle control; topic data through the schema's decoder for its topic | Every frame the session sends is one valid frame with no sequence gap; the session counts exactly the frames the sink took |

The session target skips COBS and the CRC, which the frame reader target covers, so its inputs reach the frame bodies. Its input format is in `test/fuzz/session_input.hpp`.

The build writes the seed corpus from the schema and the protocol, so it never falls out of step: a valid frame of each request the MCU answers, topic data for every schema topic, and a whole session. `ctest --preset fuzz` runs each target for 60 s, three at a time, and fails on any sanitizer report, failed check or input that runs over 10 s; CI runs it on every push:

```bash
cmake --preset fuzz
cmake --build --preset fuzz
ctest --preset fuzz
```

For a shorter or longer bounded run, set `VK_FUZZ_SECONDS` (`cmake --preset fuzz -DVK_FUZZ_SECONDS=600`). Longer runs are started by hand from the build directory; what the fuzzer finds accumulates in `corpus/` and carries over to the next run:

```bash
cd build/fuzz/test/fuzz
./vk_fuzz_session -max_total_time=3600 -jobs=4 -max_len=4096 -artifact_prefix=artifacts/ corpus/session seeds/session
```

A finding leaves its input in `build/fuzz/test/fuzz/artifacts/`. Run the target on that file alone to reproduce it, fix the bug, and add the case as a unit test in `test/unit/link_test.cpp`, so the `test` preset keeps checking it. A memory safety bug in a Link decoder is a security issue (`SECURITY.md`): report it privately rather than in a public issue.

## Bring-up image

The bring-up test build (L4) runs the hardware bring-up cases at boot instead of the application: it reads the ICM20948's WHO_AM_I over software I2C and measures the IMU read rate. It never configures actuator outputs, so it is safe with motors connected. Build it in the devcontainer, then run the bring-up runner on the host, which flashes it over SWD, reads the result block back and exits non-zero if a case fails or no results arrive:

```bash
cmake --preset c50c-v1_1-bringup
cmake --build --preset c50c-v1_1-bringup
tools/bringup/run_bringup.py build/c50c-v1_1-bringup/app/vehkernel.elf
```

Bring-up cases that move actuators run in the main firmware's test mode instead ([Driving the wheels](driving.md#motor-bring-up)).

## Hardware checks

Each milestone ends with a hardware check on the board, recorded on its spec's last ticket (`docs/specs/`). The checks a person can repeat:

| Check | Guide |
|---|---|
| Serial Link at 200 Hz for a minute | [Link monitor](link-monitor.md#checks) |
| Attitude follows the board | [Link monitor](link-monitor.md#attitude) |
| Parameters survive a power cycle | [Link monitor](link-monitor.md#parameters) |
| Motor and encoder mapping | [Driving the wheels](driving.md#motor-bring-up) |
| Fault stop and hardware watchdog | [Driving the wheels](driving.md#fault-stop-check) |
