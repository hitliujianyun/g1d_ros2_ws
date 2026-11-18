import time

from g1d_core.localization import LocalizationHealth, LocalizationParams
from g1d_core.shutdown import context_closing


def test_not_localized_before_a_good_registration():
    h = LocalizationHealth()
    assert h.status(0.0) == (False, ["no_registration"])
    h.update(0.0, False, 0, 1000, 1.0)
    assert h.status(0.1) == (False, ["not_converged"])
    h.update(0.5, True, 900, 1000, 0.1)
    assert h.status(0.6) == (True, [])


def test_slow_registration_keeps_localization_stale_loses_it():
    h = LocalizationHealth(LocalizationParams(slow_s=3.0, stale_s=10.0))
    h.update(0.0, True, 990, 1000, 0.1)
    assert h.status(2.0) == (True, [])
    assert h.status(4.0) == (True, ["registration_slow"])
    assert h.status(11.0) == (False, ["registration_stale"])
    h.update(11.5, True, 990, 1000, 0.1)
    assert h.status(11.6) == (True, [])


def test_consecutive_failures_lose_localization():
    h = LocalizationHealth(LocalizationParams(max_failed=4))
    h.update(0.0, True, 990, 1000, 0.1)
    for k in range(3):
        h.update(0.5 * (k + 1), True, 300, 1000, 0.5)              # low inliers
    assert h.status(1.6) == (True, ["low_inliers"])
    h.update(2.0, False, 0, 1000, 1.0)
    assert h.status(2.1) == (False, ["not_converged", "registration_failing"])
    h.update(2.5, True, 990, 1000, 0.1)                            # one good result recovers
    assert h.status(2.6) == (True, [])


def test_reset_waits_for_confirmation():
    h = LocalizationHealth()
    h.update(0.0, True, 990, 1000, 0.1)
    h.reset()
    assert h.status(0.1)[0] is False
    h.update(0.5, True, 990, 1000, 0.1)
    assert h.status(0.6)[0] is True


def test_context_closing():
    assert context_closing(lambda: False, timeout=0.5)
    t0 = time.monotonic()
    assert not context_closing(lambda: True, timeout=0.2, poll=0.02)
    assert time.monotonic() - t0 >= 0.2
    t_close = time.monotonic() + 0.1
    assert context_closing(lambda: time.monotonic() < t_close, timeout=1.0, poll=0.02)
