# SPDX-License-Identifier: GPL-3.0-or-later
"""replay_board_log: replays a log recorded on the C50C V1.1 and checks the match (spec 0011, ADR-0018 L2).

The log, data/c50c_v1_1_drive.ulg, is the first 30 s after the estimators' restart of a drive recorded in the
hardware check (#143): firmware 36c0488f, the estimator_replay profile over SERIAL1, a straight run, turns
and spins on the ground, no dropouts. It was cut with
`tools/log/cut.py 143-ref2.ulg c50c_v1_1_drive.ulg --duration 30`.

The board's float maths is not the host's (fused multiply-adds on the Cortex-M4 FPU, newlib's libm), so the
replay matches within TOLERANCE, not exactly. The largest difference seen was 2.6e-6 on this cut (q 1.0e-6,
position 2.6e-6 m) and 1.4e-5 on the whole 70 s drive; TOLERANCE is 1e-4, so a position off by 0.1 mm fails.

When the estimators change on purpose, record the reference again on the board, or judge the new estimate by
its metrics against the previous one, and say which in the commit.
"""

from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "log"))

from compare import compare_logs  # noqa: E402
from replay import replay  # noqa: E402

BOARD_LOG = Path(__file__).resolve().parent / "data" / "c50c_v1_1_drive.ulg"
TOLERANCE = 1e-4


def test_replay_board_log(tmp_path: Path):
    replayed = tmp_path / "replayed.ulg"
    # The spec 0011 board log was recorded before EST_PRIMARY existed, so override
    # EST_PRIMARY=1 (MAHONY) to replay with the Mahony estimator that generated the logged estimates.
    replay(BOARD_LOG, replayed, params={"EST_PRIMARY": 1})
    assert compare_logs(BOARD_LOG, replayed, tolerance=TOLERANCE)
