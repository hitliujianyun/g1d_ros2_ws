#!/usr/bin/env python3
"""CI check: committed text files use LF line endings and carry no UTF-8 BOM (submodules skipped).

Line endings are checked on the index (`git ls-files --eol`), so the result does not depend on the
working-tree conversion of the local checkout.
"""
import subprocess
import sys


def main():
    out = subprocess.run(["git", "ls-files", "--eol", "-s", "-z"], check=True,
                         capture_output=True).stdout.decode()
    bad = []
    for rec in out.split("\0"):
        if not rec:
            continue
        meta, eol, name = rec.split("\t", 2)
        mode, eol_index = meta.split()[0], eol.split()[0]
        if mode == "160000" or name.endswith(".ps1"):   # submodule / Windows script
            continue
        if eol_index in ("i/crlf", "i/mixed"):
            bad.append(f"{name}: {eol_index}")
            continue
        if eol_index == "i/-text":
            continue
        try:
            with open(name, "rb") as f:
                if f.read(3) == b"\xef\xbb\xbf":
                    bad.append(f"{name}: UTF-8 BOM")
        except OSError:
            pass
    for b in bad:
        print(b)
    print(f"checked text files: {'FAIL' if bad else 'OK'}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
