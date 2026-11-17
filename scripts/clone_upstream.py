#!/usr/bin/env python3
"""Clone / update the upstream repositories listed in upstream.lock into _upstream/<name>.

    python3 scripts/clone_upstream.py                 # all
    python3 scripts/clone_upstream.py FAST_LIO small_gicp
    python3 scripts/clone_upstream.py --list          # name, ref, local commit

Idempotent: existing clones are fetched and checked out at the requested ref.
"""
import argparse
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOCK = os.path.join(ROOT, "upstream.lock")
DEST = os.path.join(ROOT, "_upstream")


def read_lock():
    entries = []
    with open(LOCK, encoding="utf-8") as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if line:
                name, url, ref = line.split()
                entries.append((name, url, None if ref == "-" else ref))
    return entries


def git(*args, cwd=None):
    return subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True,
                          text=True).stdout.strip()


def head(path):
    try:
        return git("rev-parse", "--short", "HEAD", cwd=path)
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "-"


def sync(name, url, ref):
    path = os.path.join(DEST, name)
    if not os.path.isdir(os.path.join(path, ".git")):
        cmd = ["clone", "--recursive", url, path]
        if ref:
            cmd[1:1] = ["--branch", ref]
        git(*cmd)
    else:
        git("fetch", "--tags", "origin", cwd=path)
        if ref:
            git("checkout", ref, cwd=path)
            git("pull", "--ff-only", cwd=path)
    print(f"{name:<28} {ref or '(default)':<12} {head(path)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("names", nargs="*")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()
    entries = read_lock()
    known = {e[0] for e in entries}
    unknown = set(args.names) - known
    if unknown:
        sys.exit(f"not in upstream.lock: {', '.join(sorted(unknown))}")
    os.makedirs(DEST, exist_ok=True)
    for name, url, ref in entries:
        if args.names and name not in args.names:
            continue
        if args.list:
            print(f"{name:<28} {ref or '(default)':<12} {head(os.path.join(DEST, name))}")
        else:
            sync(name, url, ref)


if __name__ == "__main__":
    main()
