#!/usr/bin/env python3
"""Download the STL meshes referenced by urdf/g1_d_with_dex1_1.urdf from the Hugging Face dataset
(pinned revision, see UPSTREAM.md) into the package's meshes/ directory. Meshes are only needed for
RViz / Foxglove display; TF works without them.

    python3 src/g1d_description/scripts/fetch_meshes.py [--dest DIR]
"""
import argparse
import os
import re
import urllib.request

REVISION = "27b13d380531c0c6ea6ab109511ee4872594e271"
BASE = ("https://huggingface.co/datasets/unitreerobotics/unitree_sim_isaaclab_usds/resolve/"
        f"{REVISION}/assets/urdf/g1_d_description/meshes/meshes/")
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dest", default=os.path.join(PKG, "meshes"))
    args = ap.parse_args()
    with open(os.path.join(PKG, "urdf", "g1_d_with_dex1_1.urdf"), encoding="utf-8") as f:
        names = sorted(set(re.findall(r"package://g1d_description/meshes/([^\"]+)", f.read())))
    os.makedirs(args.dest, exist_ok=True)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler())
    for i, name in enumerate(names, 1):
        out = os.path.join(args.dest, name)
        if os.path.exists(out) and os.path.getsize(out) > 0:
            continue
        with opener.open(BASE + name, timeout=60) as r, open(out + ".part", "wb") as f:
            f.write(r.read())
        os.replace(out + ".part", out)
        print(f"[{i}/{len(names)}] {name}")
    print(f"meshes in {args.dest}")


if __name__ == "__main__":
    main()
