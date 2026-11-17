import json

import numpy as np

from g1d_core.pcd import write_pcd
from g1d_tools import map_compare


def test_map_compare_cli(tmp_path, capsys):
    rng = np.random.default_rng(0)
    walls = []
    for z in np.arange(0.0, 1.5, 0.05):
        walls += [(x, -2.0, z) for x in np.arange(-3, 3, 0.05)] + [(-3.0, y, z) for y in np.arange(-2, 2, 0.05)]
        walls += [(1.0, y, z) for y in np.arange(-0.5, 0.5, 0.05)]
    a = np.array(walls)
    (tmp_path / "a").mkdir()
    write_pcd(str(tmp_path / "a" / "map.pcd"), a)
    write_pcd(str(tmp_path / "b.pcd"), a + rng.normal(0, 0.005, a.shape))
    map_compare.main([str(tmp_path / "a"), str(tmp_path / "b.pcd")])
    out = json.loads(capsys.readouterr().out)
    assert out["offset_m"] < 0.02 and out["inlier_ratio"] > 0.95
