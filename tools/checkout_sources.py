#!/usr/bin/env python3
"""Fetch or verify pinned MoveIt/UR5 revisions. / 检出或核对固定源码版本。"""

import argparse
import json
import subprocess
from pathlib import Path


def git(*args, **kwargs):
    return subprocess.check_output(["git", *args], cwd=kwargs.get("cwd"), universal_newlines=True).strip()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("src", type=Path, help="catkin workspace src directory")
    parser.add_argument("--check", action="store_true", help="verify existing checkouts without cloning")
    args = parser.parse_args()
    target = args.src.expanduser().resolve()
    manifest = json.loads((Path(__file__).resolve().parent / "source-revisions.json").read_text())
    if not args.check:
        target.mkdir(parents=True, exist_ok=True)
    for name, spec in manifest.items():
        checkout = target / name
        if not checkout.exists():
            if args.check:
                raise SystemExit(f"Missing {checkout}")
            git("clone", spec["url"], str(checkout))
            git("checkout", "--detach", spec["commit"], cwd=checkout)
        actual = git("rev-parse", "HEAD", cwd=checkout)
        if actual != spec["commit"]:
            raise SystemExit(f"{name}: expected {spec['commit']}, found {actual}; inspect it manually")
        print(f"{name}: {actual}")


if __name__ == "__main__":
    main()
