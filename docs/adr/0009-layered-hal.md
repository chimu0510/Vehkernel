# Layered HAL; the form of polymorphism follows when the choice is made

The firmware must support several MCUs, SITL and unit tests, and switch vehicles and frames at runtime. The HAL is therefore layered, and the form of polymorphism follows when a choice is made: concrete classes checked by C++20 concepts for choices fixed at build time, virtual interfaces only for choices made at runtime. The board is fixed at build time, one firmware image per board; the vehicle type, the frame and the use of each port are set at runtime through parameters (ADR-0011, ADR-0013).

**Layers**

Top to bottom: module layer → driver layer → interface layer → platform layer. Dependencies point downward only. A layer may skip the driver layer and use interface-layer capabilities directly, for example a Link transport using a UART or actuator control using `PwmOutput`; no layer may bypass the interface layer to reach the platform layer.

Two shared parts sit outside the layering. Every layer, including the platform layer, may depend on `core/` (bus, work queues, parameter runtime, timing monitor); `core/` itself depends only on FreeRTOS and the interface layer, reaching hardware such as clocks and Flash through interface-layer contracts, never through the platform, driver or module layers. The math libraries in `lib/` depend on no layer and not on `core/` (ADR-0014); every layer may use them, as SITL's simulated sensors do.

**Forms of polymorphism**

- **Platform peripherals**: one MCU per build, so concrete classes whose contracts are checked by C++20 concepts. STM32, SITL and unit tests switch implementations through build configuration; drivers are not templates.
- **Between drivers and their consumers**: no interface at all; they are decoupled by the bus.
- **Choices made at runtime**: Link transports and protocols, vehicle models and frames use virtual interfaces.

Because drivers and modules are not templates, they reach a platform's concrete class through a small handle of the interface layer: a type-erased device handle bound to any class that satisfies the concept, such as `I2cDevice`, `DisplayDevice` and `FlashSectors`, or a capability interface. The concept still checks the platform class at build time; the handle costs one indirect call per access.

The interface layer stays small and defines only what generic drivers need. Highly platform-specific features such as timers are offered as capabilities, for example `PwmOutput` and `QuadratureEncoder`.

The platform layer is built on ST LL, using ST HAL only where needed (for example USB).

## Considered Options

- **Virtual interfaces for the whole platform layer (ArduPilot)**: rejected. A build has exactly one platform, so runtime polymorphism buys nothing.
- **modm as the platform layer**: rejected. It brings its own architecture and strong lock-in. We borrow only its idea that misusing a pin fails at compile time, with our own board generator emitting typed pin constants.
- **ST HAL2**: does not support STM32F4.
