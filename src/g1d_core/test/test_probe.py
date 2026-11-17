import pytest

from g1d_core.probe import RateStats, stop_gate, stop_stats


def test_rate_and_offsets():
    rs = RateStats()
    for i in range(21):
        rs.add(100.0 + 0.05 * i, stamp=100.0 + 0.05 * i - 181.0 - (0.01 if i == 3 else 0.0))
    r = rs.report()
    assert r["count"] == 21 and r["rate_hz"] == pytest.approx(20.0)
    assert r["stamp_offset_s"] == pytest.approx(181.0)
    assert r["stamp_offset_spread_s"] == pytest.approx(0.01)
    assert RateStats().report() == {"count": 0, "rate_hz": None, "stamp_offset_s": None,
                                    "stamp_offset_spread_s": None}


def test_stop_stats_and_gate():
    rows = []
    for i in range(100):                      # 0.3 m/s until t = 1.0, then 1.5 m/s^2 braking
        t = i * 0.02
        v = 0.3 if t < 1.0 else max(0.0, 0.3 - 1.5 * (t - 1.0))
        rows.append((t, 0.3 * t, 0.0, 0.0, v, 0.0))
    s = stop_stats(rows, 0.5)
    assert s["v_at_release"] == pytest.approx(0.3)
    assert s["stop_time_s"] == pytest.approx(0.5 + 0.3 / 1.5 - 0.0133, abs=0.03)
    assert s["stop_distance_m"] > 0.1
    assert stop_gate(s)["pass"] is (s["stop_time_s"] <= 0.6)
    assert stop_gate({"stop_time_s": None})["pass"] is False
    assert stop_stats(rows, 10.0) == {"error": "no odometry"}
