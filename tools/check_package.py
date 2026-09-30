#!/usr/bin/env python3
"""Check source hashes and the dataset. / 检查源码哈希与三场景数据。"""

import hashlib
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = ROOT / "archive/local_history_20250930"
DATA = ROOT / "data/joint_usage_1"
RUN = re.compile(r"^(base|rul-aware)_(0\.8|1|1\.5)_([123])\.csv$")
SCENES = ("Full_Health_System", "Heterogeneous_Degradation_System", "Local_Degradation_System")


def check_snapshot():
    lines = (ARCHIVE / "SHA256SUMS.relative.txt").read_text(encoding="utf-8").splitlines()
    if len(lines) != 3:
        raise ValueError("Expected three original source hashes")
    for line in lines:
        expected, filename = line.split(None, 1)
        filename = filename.strip()
        if filename not in {"ur5_group.py", "RULRRTstar.cpp", "RULAwareOptimizationObjective.cpp"}:
            raise ValueError("Unexpected snapshot entry: " + filename)
        actual = hashlib.sha256((ARCHIVE / filename).read_bytes()).hexdigest()
        if actual != expected.lower():
            raise ValueError("SHA-256 mismatch: " + filename)


def check_data():
    total = 0
    for scene in SCENES:
        folder = DATA / scene
        runs = [path for path in folder.glob("*.csv") if RUN.fullmatch(path.name)]
        if len(runs) != 18:
            raise ValueError("Expected 18 runs in " + scene)
        for path in runs:
            for suffix in ("_joint_usage.csv", "_joint_loss.csv"):
                if not path.with_name(path.stem + suffix).is_file():
                    raise ValueError("Missing paired file for " + path.name)
        total += len(runs)
    if len(list((DATA / "statistics").rglob("*.csv"))) != 35:
        raise ValueError("Expected 35 saved statistics CSVs")
    if not (DATA / "Full_Health_System/FHS_rul_sum_summary.csv").is_file():
        raise ValueError("Missing FHS sum summary")
    return total


def main():
    check_snapshot()
    total = check_data()
    for name in ("planner/src/ompl/geometric/planners/rrt/RULRRTstar.h",
                 "planner/src/ompl/geometric/planners/rrt/src/RULRRTstar.cpp",
                 "planner/src/ompl/base/objectives/RULAwareOptimizationObjective.h",
                 "planner/src/ompl/base/objectives/src/RULAwareOptimizationObjective.cpp",
                 "experiment/run_experiment.py", "ros/ur5_moveit_config/config/ompl_planning.yaml"):
        if not (ROOT / name).is_file():
            raise ValueError("Missing source/config: " + name)
    print("Package OK / 包检查通过: 3 source hashes / 源码哈希, {} runs / 次运行, 108 paired CSVs / 配套文件, 36 summaries / 汇总".format(total))


if __name__ == "__main__":
    main()
