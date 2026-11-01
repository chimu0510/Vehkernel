# SPDX-License-Identifier: GPL-3.0-or-later
# Counter published only by the board examples (spec 0013, ADR-0025).
# Queued so a subscriber woken by each publication reads every sample (ADR-0008).
@queue 4
@rate 10
uint64 timestamp
uint32 count
