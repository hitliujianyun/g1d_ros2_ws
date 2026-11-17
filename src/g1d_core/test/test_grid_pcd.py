import numpy as np
import pytest

from g1d_core.grid import (FREE, OCCUPIED, UNKNOWN, GridParams, build_grid, load_map,
                           occupancy_scores, rasterize_boxes, save_map)
from g1d_core.pcd import read_pcd, read_poses_txt, transform, voxel_downsample, write_pcd, xyz


def sample_box_surface(b, step=0.03):
    x0, y0, z0, x1, y1, z1 = b
    pts = []
    for x in np.arange(x0, x1 + 1e-9, step):
        for z in np.arange(z0, z1 + 1e-9, step):
            pts += [(x, y0, z), (x, y1, z)]
    for y in np.arange(y0, y1 + 1e-9, step):
        for z in np.arange(z0, z1 + 1e-9, step):
            pts += [(x0, y, z), (x1, y, z)]
        for x in np.arange(x0, x1 + 1e-9, step):
            pts += [(x, y, z1)] + ([(x, y, z0)] if z0 > 0 else [])
    return pts


def floor(x0, y0, x1, y1, step=0.04, holes=()):
    pts = []
    for x in np.arange(x0, x1, step):
        for y in np.arange(y0, y1, step):
            if not any(h[0] <= x <= h[2] and h[1] <= y <= h[3] for h in holes):
                pts.append((x, y, 0.0))
    return pts


BOXES = [[-1.0, -1.1, 0.0, 3.0, -1.0, 1.5], [1.0, 0.2, 0.0, 1.3, 0.5, 1.0],
         [2.0, -0.6, 0.72, 2.6, 0.0, 0.76]]       # wall, box, table top (overhang)


def test_height_band_grid_with_overhang():
    pts = floor(-1.0, -1.0, 3.0, 1.0, holes=[(1.0, 0.2, 1.3, 0.5)])
    for b in BOXES:
        pts += sample_box_surface(b)
    pts += [(0.0, 0.8, 2.3)]                          # ceiling-height point: ignored
    g = build_grid(np.array(pts), GridParams(resolution=0.05))
    assert g.value_at(1.15, 0.2) == OCCUPIED          # box face
    assert g.value_at(2.3, -0.3) == OCCUPIED          # table top, though floor is seen below it
    assert g.value_at(0.5, 0.0) == FREE
    assert g.value_at(0.0, 0.8) == FREE               # ceiling point does not occupy
    assert g.value_at(10.0, 10.0) == UNKNOWN
    truth = rasterize_boxes(BOXES, g)
    precision, recall = occupancy_scores(g, truth)
    assert precision > 0.95 and recall > 0.9


def test_map_files_roundtrip(tmp_path):
    pts = np.array(floor(0, 0, 1, 1) + sample_box_surface([0.4, 0.4, 0.0, 0.6, 0.6, 1.0]))
    g = build_grid(pts)
    path = save_map(g, str(tmp_path), "map")
    g2 = load_map(path)
    assert g2.data.shape == g.data.shape and g2.origin == pytest.approx(g.origin)
    np.testing.assert_array_equal(g2.data, g.data)
    assert (tmp_path / "map.pgm").read_bytes().startswith(b"P5\n")


def test_pcd_roundtrip_and_helpers(tmp_path):
    pts = np.random.default_rng(1).uniform(-5, 5, (1000, 3))
    write_pcd(str(tmp_path / "a.pcd"), pts, intensity=np.arange(1000))
    f = read_pcd(str(tmp_path / "a.pcd"))
    np.testing.assert_allclose(xyz(f), pts, atol=1e-5)
    assert f["intensity"][-1] == 999
    (tmp_path / "b.pcd").write_text("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                                     "WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n1 2 3\n4 5 6\n")
    np.testing.assert_allclose(xyz(read_pcd(str(tmp_path / "b.pcd"))), [[1, 2, 3], [4, 5, 6]])
    d = voxel_downsample(np.array([[0.01, 0, 0], [0.03, 0, 0], [0.5, 0, 0]]), 0.1)
    assert len(d) == 2
    t = np.eye(4)
    t[:3, 3] = [1, 2, 3]
    np.testing.assert_allclose(transform(np.zeros((1, 3)), t), [[1, 2, 3]])
    (tmp_path / "poses.txt").write_text("0.pcd 1 2 3 1 0 0 0\n1.pcd 4 5 6 0 0 0 1\n")
    names, poses = read_poses_txt(str(tmp_path / "poses.txt"))
    assert names == ["0.pcd", "1.pcd"] and poses[0].tolist() == [1, 2, 3, 0, 0, 0, 1]
