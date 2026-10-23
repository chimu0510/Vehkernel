# SPDX-License-Identifier: GPL-3.0-or-later
"""STM32F4 clock tree from a board's crystal and target system clock.

Solves the main PLL (M, N, P, Q) for an exact system clock and an exact
48 MHz USB clock, then picks the APB prescalers and Flash wait states, using
the STM32F405/407 limits of the reference manual (RM0090) at 2.7-3.6 V.
"""

from __future__ import annotations

from dataclasses import dataclass

CLOCK_KEYS = {"hse_hz", "sysclk_hz"}

_MHZ = 1_000_000
HSE_RANGE = (4 * _MHZ, 26 * _MHZ)
VCO_IN_RANGE = (1 * _MHZ, 2 * _MHZ)
VCO_OUT_RANGE = (100 * _MHZ, 432 * _MHZ)
SYSCLK_MAX = 168 * _MHZ
APB1_MAX = 42 * _MHZ
APB2_MAX = 84 * _MHZ
USB_HZ = 48 * _MHZ
FLASH_WAIT_STEP = 30 * _MHZ  # one wait state per 30 MHz of HCLK at 2.7-3.6 V


@dataclass(frozen=True)
class Clock:
    hse_hz: int
    sysclk_hz: int
    pll_m: int
    pll_n: int
    pll_p: int
    pll_q: int
    apb1_div: int
    apb2_div: int
    flash_wait_states: int

    @property
    def apb1_hz(self) -> int:
        return self.sysclk_hz // self.apb1_div

    @property
    def apb2_hz(self) -> int:
        return self.sysclk_hz // self.apb2_div

    @property
    def apb1_timer_hz(self) -> int:
        return self.apb1_hz * (1 if self.apb1_div == 1 else 2)

    @property
    def apb2_timer_hz(self) -> int:
        return self.apb2_hz * (1 if self.apb2_div == 1 else 2)


def solve_clock(hse_hz: int, sysclk_hz: int) -> Clock | str:
    """The clock tree, or a reason why none exists."""
    if not HSE_RANGE[0] <= hse_hz <= HSE_RANGE[1]:
        return f"hse_hz {hse_hz} is outside the {HSE_RANGE[0] // _MHZ}-{HSE_RANGE[1] // _MHZ} MHz crystal range"
    if not 0 < sysclk_hz <= SYSCLK_MAX:
        return f"sysclk_hz {sysclk_hz} is above the {SYSCLK_MAX // _MHZ} MHz maximum"

    solutions = []
    for m in range(2, 64):
        if hse_hz % m or not VCO_IN_RANGE[0] <= hse_hz // m <= VCO_IN_RANGE[1]:
            continue
        vco_in = hse_hz // m
        for p in (2, 4, 6, 8):
            vco_out = sysclk_hz * p
            if vco_out % vco_in or not VCO_OUT_RANGE[0] <= vco_out <= VCO_OUT_RANGE[1]:
                continue
            n = vco_out // vco_in
            if not 50 <= n <= 432 or vco_out % USB_HZ:
                continue
            q = vco_out // USB_HZ
            if 2 <= q <= 15:
                # Prefer the highest VCO input (least jitter), then the lowest P.
                solutions.append((-vco_in, p, m, n, q))
    if not solutions:
        return (f"no PLL setting gives exactly {sysclk_hz} Hz system clock and a 48 MHz USB clock "
                f"from a {hse_hz} Hz crystal")
    _, p, m, n, q = min(solutions)

    def divider(limit: int) -> int:
        return next(d for d in (1, 2, 4, 8, 16) if sysclk_hz // d <= limit)

    return Clock(hse_hz=hse_hz, sysclk_hz=sysclk_hz, pll_m=m, pll_n=n, pll_p=p, pll_q=q,
                 apb1_div=divider(APB1_MAX), apb2_div=divider(APB2_MAX),
                 flash_wait_states=(sysclk_hz - 1) // FLASH_WAIT_STEP)
