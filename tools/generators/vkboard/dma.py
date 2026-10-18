"""DMA stream and channel assignment for STM32F4-style DMA (ADR-0011).

The F4 has no DMAMUX: each request can use only the few stream/channel pairs
listed in the MCU database. Requests are placed in priority order (higher
first, then by name) with backtracking, so the result is deterministic.

Requests come from two places:

- Implied by pins: USART/UART TX and RX pins, and both directions of every SPI
  peripheral that has a pin.
- Listed in the board's ``dma`` array, which can add requests (timer, ADC, ...)
  and annotate any request with ``priority``, ``dma = false`` or ``share = true``.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

from .diagnostics import Diagnostics, Loc

DMA_KEYS = {"request", "priority", "dma", "share"}

_SERIAL_FUNC_RE = re.compile(r"^U(S)?ART\d+_(TX|RX)$")
_SPI_FUNC_RE = re.compile(r"^(SPI\d+)_")


@dataclass(frozen=True)
class Candidate:
    controller: str
    stream: int
    channel: int

    @property
    def stream_key(self) -> tuple[str, int]:
        return self.controller, self.stream

    def __str__(self) -> str:
        return f"{self.controller} S{self.stream} C{self.channel}"


@dataclass
class DmaRequest:
    name: str
    candidates: tuple[Candidate, ...]
    priority: int = 0
    share: bool = False
    enabled: bool = True
    loc: Loc | None = None
    implied: bool = False


@dataclass(frozen=True)
class DmaAssignment:
    request: str
    candidate: Candidate | None  # None when DMA is disabled for the request
    priority: int
    share: bool
    shared_with: tuple[str, ...] = field(default=())
    candidates: tuple[Candidate, ...] = field(default=())


def implied_requests(funcs: list[str], dma_map: dict) -> list[str]:
    """Requests a board needs DMA for by default, from its alternate-function pins."""
    names = set()
    for func in funcs:
        if _SERIAL_FUNC_RE.match(func) and func in dma_map:
            names.add(func)
        spi = _SPI_FUNC_RE.match(func)
        if spi:
            names.update(n for n in (f"{spi.group(1)}_RX", f"{spi.group(1)}_TX") if n in dma_map)
    return sorted(names)


def collect_requests(raw: list, locs: list[Loc], funcs: list[str], dma_map: dict,
                     diag: Diagnostics) -> list[DmaRequest]:
    def candidates(name: str) -> tuple[Candidate, ...]:
        return tuple(Candidate(c["controller"], c["stream"], c["channel"]) for c in dma_map[name])

    requests = {name: DmaRequest(name, candidates(name), implied=True)
                for name in implied_requests(funcs, dma_map)}
    seen: dict[str, Loc] = {}
    for entry, line in zip(raw, locs):
        if not isinstance(entry, dict):
            diag.error("each dma entry must be an inline table", line)
            continue
        unknown = sorted(set(entry) - DMA_KEYS)
        for key in unknown:
            diag.error(f"unknown key '{key}' in dma entry", line)
        name = entry.get("request")
        if not isinstance(name, str):
            diag.error("dma entry is missing 'request'", line)
            continue
        if unknown:
            continue
        if name in seen:
            diag.error(f"DMA request {name} is already listed ({seen[name].refer(line)})", line)
            continue
        seen[name] = line
        if name not in dma_map:
            diag.error(f"DMA request {name} does not exist on this MCU", line)
            continue
        priority, enabled, share = entry.get("priority", 0), entry.get("dma", True), entry.get("share", False)
        if not isinstance(priority, int) or isinstance(priority, bool) or priority < 0:
            diag.error(f"'priority' of DMA request {name} must be a non-negative integer", line)
            continue
        if not isinstance(enabled, bool) or not isinstance(share, bool):
            diag.error(f"'dma' and 'share' of DMA request {name} must be true or false", line)
            continue
        request = requests.get(name) or DmaRequest(name, candidates(name))
        request.priority, request.enabled, request.share, request.loc = priority, enabled, share, line
        requests[name] = request
    return [requests[name] for name in sorted(requests)]


def _order(requests: list[DmaRequest]) -> list[DmaRequest]:
    return sorted((r for r in requests if r.enabled), key=lambda r: (-r.priority, r.name))


def solve(requests: list[DmaRequest]) -> dict[str, Candidate] | None:
    """First assignment found in priority and candidate order, or None if none exists."""
    order = _order(requests)
    users: dict[tuple[str, int], list[DmaRequest]] = {}
    chosen: dict[str, Candidate] = {}

    def fits(request: DmaRequest, candidate: Candidate) -> bool:
        current = users.get(candidate.stream_key, [])
        return not current or (request.share and all(u.share for u in current))

    def place(index: int) -> bool:
        if index == len(order):
            return True
        request = order[index]
        for candidate in request.candidates:
            if fits(request, candidate):
                users.setdefault(candidate.stream_key, []).append(request)
                chosen[request.name] = candidate
                if place(index + 1):
                    return True
                users[candidate.stream_key].pop()
                del chosen[request.name]
        return False

    return dict(chosen) if place(0) else None


def minimal_conflict(requests: list[DmaRequest]) -> list[DmaRequest]:
    """A smallest-by-deletion set of requests that still cannot all be placed."""
    conflict = _order(requests)
    for request in reversed(_order(requests)):
        trial = [r for r in conflict if r is not request]
        if solve(trial) is None:
            conflict = trial
    return conflict


def assign(requests: list[DmaRequest], diag: Diagnostics) -> list[DmaAssignment]:
    solution = solve(requests)
    if solution is None:
        conflict = minimal_conflict(requests)
        names = ", ".join(r.name for r in conflict)
        streams = sorted({c.stream_key for r in conflict for c in r.candidates})
        stream_text = ", ".join(f"{ctrl} S{stream}" for ctrl, stream in streams)
        line = next((r.loc for r in conflict if r.loc), None)
        diag.error(f"DMA requests {names} cannot all get a stream; they compete for {stream_text} "
                   f"(set 'share = true' on all of them, or 'dma = false' on one)", line)
        return []

    by_stream: dict[tuple[str, int], list[str]] = {}
    for name, candidate in solution.items():
        by_stream.setdefault(candidate.stream_key, []).append(name)
    result = []
    for r in requests:
        candidate = solution.get(r.name)
        shared = tuple(sorted(n for n in by_stream.get(candidate.stream_key, []) if n != r.name)) if candidate else ()
        result.append(DmaAssignment(r.name, candidate, r.priority, r.share, shared, r.candidates))
    return result
