"""Mapping repeatability: compare two maps of the same area (both sessions started at the same floor
marker with the same heading, so the map frames should coincide).

    ros2 run g1d_tools map_compare ~/g1d_maps/run1 ~/g1d_maps/run2 [--voxel 0.1]
Reports the residual offset between the maps (should be ~0) and the surface agreement (RMSE / P90).
"""
import argparse
import json
import os

from g1d_core.metrics import map_consistency
from g1d_core.pcd import read_pcd, xyz


def load(path):
    return xyz(read_pcd(os.path.join(path, "map.pcd") if os.path.isdir(path) else path))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_a")
    ap.add_argument("map_b")
    ap.add_argument("--voxel", type=float, default=0.1)
    ap.add_argument("--max-corr", type=float, default=0.5)
    args = ap.parse_args(argv)
    r = map_consistency(load(args.map_a), load(args.map_b), voxel=args.voxel, max_corr=args.max_corr)
    out = {"offset_m": round(r.offset_m, 4), "offset_deg": round(r.offset_deg, 3), "rmse_m": round(r.rmse, 4),
           "p90_m": round(r.p90, 4), "inlier_ratio": round(r.inlier_ratio, 3)}
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
