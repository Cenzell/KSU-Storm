"""Match result storage and statistics.

Results are persisted as JSON in ~/.ksu_storm/match_history.json so they
survive across driver-station restarts.
"""

from __future__ import annotations

import json
import logging
import time
import uuid
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import List, Optional

logger = logging.getLogger(__name__)

HISTORY_PATH = Path.home() / ".ksu_storm" / "match_history.json"


@dataclass
class MatchResult:
    # Identity
    match_id: str               # UUID string, auto-generated
    timestamp: float            # Unix time of save
    match_label: str            # e.g. "Ranking 3", "Playoff SF1"
    match_type: str             # "Ranking" | "Playoff"

    # Teams
    alliance: str               # "RED" | "BLUE"
    opponent_team: str

    # Scores
    our_score: int
    opponent_score: int
    outcome: str                # "WIN" | "LOSS" | "TIE"

    # Scoring breakdown (for our team)
    batteries_auto: int = 0
    batteries_teleop: int = 0
    auto_zone_exit: bool = False
    jumpstarts_auto: int = 0
    jumpstarts_teleop: int = 0
    wheel_time_auto_s: int = 0
    wheel_time_teleop_s: int = 0
    climb: str = "None"         # "None" | "Line" | "High"
    minor_penalties: int = 0
    major_penalties: int = 0

    notes: str = ""

    @classmethod
    def create(
        cls,
        match_label: str,
        match_type: str,
        alliance: str,
        opponent_team: str,
        our_score: int,
        opponent_score: int,
        **kwargs,
    ) -> "MatchResult":
        if our_score > opponent_score:
            outcome = "WIN"
        elif our_score < opponent_score:
            outcome = "LOSS"
        else:
            outcome = "TIE"
        return cls(
            match_id=str(uuid.uuid4())[:8],
            timestamp=time.time(),
            match_label=match_label,
            match_type=match_type,
            alliance=alliance,
            opponent_team=opponent_team,
            our_score=our_score,
            opponent_score=opponent_score,
            outcome=outcome,
            **kwargs,
        )


class MatchHistory:
    """In-memory list of match results backed by a JSON file."""

    def __init__(self, matches: Optional[List[MatchResult]] = None) -> None:
        self.matches: List[MatchResult] = matches or []

    # ── statistics ────────────────────────────────────────────────────────

    @property
    def wins(self) -> int:
        return sum(1 for m in self.matches if m.outcome == "WIN")

    @property
    def losses(self) -> int:
        return sum(1 for m in self.matches if m.outcome == "LOSS")

    @property
    def ties(self) -> int:
        return sum(1 for m in self.matches if m.outcome == "TIE")

    @property
    def avg_score(self) -> float:
        if not self.matches:
            return 0.0
        return sum(m.our_score for m in self.matches) / len(self.matches)

    @property
    def high_score(self) -> int:
        return max((m.our_score for m in self.matches), default=0)

    # ── mutation ──────────────────────────────────────────────────────────

    def add(self, result: MatchResult) -> None:
        self.matches.append(result)
        self._save()
        logger.info("Match saved: %s vs %s -> %s",
                    result.match_label, result.opponent_team, result.outcome)

    def delete(self, match_id: str) -> bool:
        before = len(self.matches)
        self.matches = [m for m in self.matches if m.match_id != match_id]
        if len(self.matches) < before:
            self._save()
            return True
        return False

    # ── persistence ───────────────────────────────────────────────────────

    def _save(self) -> None:
        HISTORY_PATH.parent.mkdir(parents=True, exist_ok=True)
        try:
            with HISTORY_PATH.open("w") as f:
                json.dump([asdict(m) for m in self.matches], f, indent=2)
        except Exception as e:
            logger.error("Failed to save match history: %s", e)

    @classmethod
    def load(cls) -> "MatchHistory":
        if not HISTORY_PATH.exists():
            return cls()
        try:
            with HISTORY_PATH.open() as f:
                data = json.load(f)
            matches = []
            for item in data:
                try:
                    matches.append(MatchResult(**item))
                except Exception as e:
                    logger.warning("Skipping corrupt match record: %s", e)
            return cls(matches=matches)
        except Exception as e:
            logger.error("Failed to load match history: %s", e)
            return cls()


# ── scoring helper (mirrors the rules exactly) ────────────────────────────────

def calculate_score(
    batteries_auto: int = 0,
    batteries_teleop: int = 0,
    auto_zone_exit: bool = False,
    jumpstarts_auto: int = 0,
    jumpstarts_teleop: int = 0,
    wheel_time_auto_s: int = 0,
    wheel_time_teleop_s: int = 0,
    climb: str = "None",
    minor_penalties: int = 0,
    major_penalties: int = 0,
) -> dict:
    """Return a full scoring breakdown dict.

    Rules references:
    - §3.3.1  Autonomous: batteries=8 pts, KJ×2, zone_exit=3
    - §3.3.2  Battery install: teleop=5 pts; each pt adds 1 Capacity
    - §3.3.3  Points from KJ = min(KJ, Capacity)
    - §3.3.4  Wheel: 1 KJ per 2s at grid freq (auto KJ doubled)
    - §3.3.5  Jumpstart: 5 KJ each (auto KJ doubled), 30s cooldown
    - §3.3.6  Climb: 10 pts on line, +10 if 12"+ high
    - §4.3.2.3 Penalties: -3 minor, -8 major
    """
    # Battery points and capacity
    battery_pts_auto  = batteries_auto  * 8
    battery_pts_tele  = batteries_teleop * 5
    battery_pts       = battery_pts_auto + battery_pts_tele
    capacity          = battery_pts  # 1 capacity per point earned

    # KJ from jumpstarts (auto counts double)
    kj_jumpstart_auto = jumpstarts_auto  * 5 * 2
    kj_jumpstart_tele = jumpstarts_teleop * 5
    kj_jumpstart      = kj_jumpstart_auto + kj_jumpstart_tele

    # KJ from wheel (1 KJ per 2 s; auto counts double)
    kj_wheel_auto     = (wheel_time_auto_s  // 2) * 2
    kj_wheel_tele     = (wheel_time_teleop_s // 2)
    kj_wheel          = kj_wheel_auto + kj_wheel_tele

    total_kj          = kj_jumpstart + kj_wheel
    kj_pts            = min(total_kj, capacity)

    # Other bonuses
    auto_exit_pts     = 3 if auto_zone_exit else 0
    climb_pts         = {"None": 0, "Line": 10, "High": 20}.get(climb, 0)

    # Penalties (applied to our score)
    penalty_pts       = (minor_penalties * 3) + (major_penalties * 8)

    total = max(0, battery_pts + kj_pts + auto_exit_pts + climb_pts - penalty_pts)

    return {
        "battery_pts":       battery_pts,
        "battery_pts_auto":  battery_pts_auto,
        "battery_pts_tele":  battery_pts_tele,
        "capacity":          capacity,
        "kj_jumpstart":      kj_jumpstart,
        "kj_wheel":          kj_wheel,
        "total_kj":          total_kj,
        "kj_pts":            kj_pts,
        "auto_exit_pts":     auto_exit_pts,
        "climb_pts":         climb_pts,
        "penalty_pts":       penalty_pts,
        "total":             total,
    }
