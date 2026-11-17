"""Maneuver power parsing and scaling."""
from maneuver_power import (
    GAIN_FLOOR,
    from_env,
    gain_scale,
    normalize_power,
    power_pct,
    scale,
    spoken_power,
)


def main():
    assert abs(normalize_power(50) - 0.5) < 1e-9
    assert abs(normalize_power(0.5) - 0.5) < 1e-9
    assert abs(normalize_power(100) - 1.0) < 1e-9
    assert normalize_power(0) == 0.05
    assert abs(power_pct(50) - 50.0) < 1e-9
    assert abs(scale(0.15, 50) - 0.075) < 1e-9
    assert abs(gain_scale(1.0) - 1.0) < 1e-9
    assert abs(gain_scale(0.5) - (GAIN_FLOOR + 0.5 * (1.0 - GAIN_FLOOR))) < 1e-9
    p, rest = spoken_power("rendezvous with otter_k with 50% power")
    assert abs(p - 0.5) < 1e-9
    assert "50" not in rest and "power" not in rest.lower()
    p, _ = spoken_power("swim north at half power")
    assert abs(p - 0.5) < 1e-9
    p, _ = spoken_power("coverage at 25 percent power")
    assert abs(p - 0.25) < 1e-9
    assert spoken_power("rendezvous with otter_k")[0] is None
    assert abs(from_env({"UDRONE_POWER_PCT": "40"}) - 0.4) < 1e-9
    print("maneuver power ok")


if __name__ == "__main__":
    main()
