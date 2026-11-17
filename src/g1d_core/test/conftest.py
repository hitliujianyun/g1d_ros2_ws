import os

import pytest


def _find_description_dir():
    env = os.environ.get("G1D_DESCRIPTION_DIR")
    if env:
        return env
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        cand = os.path.join(d, "g1d_description")
        if os.path.isdir(os.path.join(cand, "urdf")):
            return cand
        d = os.path.dirname(d)
    try:  # installed workspace (colcon test from the build directory)
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("g1d_description")
    except Exception:  # noqa: BLE001
        return None


@pytest.fixture(scope="session")
def description_dir():
    d = _find_description_dir()
    if d is None:
        pytest.skip("g1d_description not found (set G1D_DESCRIPTION_DIR)")
    return d


@pytest.fixture(scope="session")
def g1d_urdf_path(description_dir):
    return os.path.join(description_dir, "urdf", "g1_d_with_dex1_1.urdf")
