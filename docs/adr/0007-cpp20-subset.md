# The firmware is written in a subset of C++20

Supporting many vehicles, frames and boards is the core of the project and needs many interface abstractions, such as vehicle models and HAL backends. PX4 and ArduPilot, which have the same goal, both chose C++. Vehkernel uses C++20 with a restricted subset:

- No exceptions, no RTTI.
- No dynamic allocation after initialization, and no allocating STL containers.
- ST LL, ST HAL and FreeRTOS stay in C and are called through `extern "C"`.

C++ also lets the generators emit typed topics such as `Subscription<BatteryStatus>`, so the compiler checks the whole schema chain.

## Considered Options

- **C11** (as in Betaflight and the vendor WHEELTEC firmware): rejected. Polymorphism would rely on function pointer tables and macros, and the bus API could only offer `void*` plus a size, with no type safety.
