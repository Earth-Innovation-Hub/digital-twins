"""Maneuver energy / power level (ROS-free).

Operator phrases like ``with 50% power`` set a fraction of full thrust and
reference pace. Lower power also softens the force-loop gains so the hull
moves more slowly and more smoothly.
"""
from __future__ import annotations

import os
import re

POWER_MIN = 0.05
POWER_MAX = 1.0
# At full power gains are nominal. At low power they ease toward this floor
# so the loop stays stable but less snappy.
GAIN_FLOOR = 0.35

_POWER_PHRASE = re.compile(
    r"(?:(?:with|at)\s+)?(?P<pct>\d+(?:\.\d+)?)\s*%\s*(?:power|thrust|energy|effort)?"
    r"|(?:(?:with|at)\s+)?(?P<pct2>\d+(?:\.\d+)?)\s+percent(?:\s+(?:power|thrust|energy|effort))?"
    r"|\b(?P<word>half|quarter|full)\s+power\b"
    r"|\bpower\s*(?:=|:)?\s*(?P<pct3>\d+(?:\.\d+)?)\s*%?",
    re.I,
)
_WORDS = {"half": 50.0, "quarter": 25.0, "full": 100.0}


def normalize_power(value) -> float:
    """Fraction in [POWER_MIN, POWER_MAX]. Accepts 0–1 or 1–100 percent."""
    if value is None or value == "":
        return 1.0
    p = float(value)
    if p > 1.0 + 1e-9:
        p = p / 100.0
    return max(POWER_MIN, min(POWER_MAX, p))


def power_pct(value) -> float:
    """Percent in [5, 100] for coral payloads and logs."""
    return round(100.0 * normalize_power(value), 3)


def spoken_power(text: str):
    """(power fraction or None, text with the power phrase removed)."""
    raw = str(text or "")
    match = _POWER_PHRASE.search(raw)
    if match is None:
        return None, raw
    if match.group("word"):
        pct = _WORDS[match.group("word").lower()]
    else:
        pct = float(match.group("pct") or match.group("pct2") or match.group("pct3"))
    if pct <= 0.0:
        return None, raw
    rest = (raw[:match.start()] + " " + raw[match.end():]).strip()
    rest = re.sub(r"\s+", " ", rest)
    return normalize_power(pct), rest


def from_env(env: dict | None = None, key: str = "UDRONE_POWER_PCT") -> float:
    src = os.environ if env is None else env
    return normalize_power(src.get(key, "100"))


def gain_scale(power: float) -> float:
    """Softens position/velocity gains at low power for smoother motion."""
    p = normalize_power(power)
    return GAIN_FLOOR + (1.0 - GAIN_FLOOR) * p


def scale(nominal: float, power: float) -> float:
    return float(nominal) * normalize_power(power)
