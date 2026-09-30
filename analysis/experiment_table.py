#!/usr/bin/env python3
"""Summarize three-scenario run CSVs. / 汇总三场景原始运行数据。"""

import argparse
import csv
import re
import statistics
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1] / "data/joint_usage_1"
SCENES = {
    "Full_Health_System": "FHS",
    "Heterogeneous_Degradation_System": "HDS",
    "Local_Degradation_System": "LDS",
}
INITIAL = {
    "FHS": ([1000.0] * 6, [1.0] * 6),
    "HDS": ([500.0, 800.0, 400.0, 600.0, 1000.0, 700.0],
            [1.55, 0.28, 2.78, 0.8, 0.13, 0.46]),
    "LDS": ([100.0] + [1000.0] * 5, [5.0] + [0.2] * 5),
}
RUN = re.compile(r"^(base|rul-aware)_(0\.8|1|1\.5)_([1-9][0-9]*)\.csv$")


def read_run(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.reader(stream))
    if not rows or any(len(row) != 16 for row in rows):
        raise ValueError("expected 16 columns in every run row: {}".format(path))
    final_rul = [float(x) for x in rows[-1][:6]]
    params = tuple(rows[0][12:16])
    usage = path.with_name(path.stem + "_joint_usage.csv")
    with usage.open(newline="", encoding="utf-8") as stream:
        task_rows = list(csv.DictReader(stream))
    if not task_rows:
        raise ValueError("empty task usage file: {}".format(usage))
    initial = [float(x) for x in rows[0][:12]]
    return final_rul, max(int(row["task"]) for row in task_rows), params, initial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-stored-summary", action="store_true")
    args = parser.parse_args()
    print("scene,planner,p,n,min_joint_mean,sum_joint_means,mean_tasks,sample_std_tasks")
    for scene, short in SCENES.items():
        folder = ROOT / scene
        runs = {}
        for path in folder.glob("*.csv"):
            match = RUN.fullmatch(path.name)
            if match:
                runs.setdefault((match.group(1), match.group(2)), []).append((int(match.group(3)), read_run(path)))
        if len(runs) != 6:
            raise ValueError("expected six planner/shape groups in {}".format(folder))
        summary = {}
        if args.check_stored_summary:
            summary_path = ROOT / "statistics" / short / "rul_summary.csv"
            with summary_path.open(newline="", encoding="utf-8") as stream:
                summary = {row["filename"]: row for row in csv.DictReader(stream)}
        for (planner, shape), group in sorted(runs.items(), key=lambda item: (item[0][1], item[0][0])):
            group.sort()
            if len(group) != 3:
                raise ValueError("expected three runs: {} {} {}".format(scene, planner, shape))
            params = {values[2] for _, values in group}
            if params != {("1.0", "1.0", "1.0", "0.0")}:
                raise ValueError("unexpected saved parameters: {} {} {}".format(scene, planner, shape))
            expected_initial = INITIAL[short][0] + INITIAL[short][1]
            if any(any(abs(x - y) > 0.005 for x, y in zip(values[3], expected_initial))
                   for _, values in group):
                raise ValueError("unexpected initial RUL/gamma: {} {} {}".format(scene, planner, shape))
            joint_means = [statistics.mean(values[0][j] for _, values in group) for j in range(6)]
            tasks = [values[1] for _, values in group]
            if args.check_stored_summary:
                row = summary[planner + "_" + shape]
                for j, mean in enumerate(joint_means, 1):
                    if abs(float(row["motor{}_mean".format(j)]) - mean) > 0.011:
                        raise ValueError("stored RUL summary differs: {} {} motor {}".format(scene, planner, j))
                if abs(float(row["tasks_completed"]) - statistics.mean(tasks)) > 0.011:
                    raise ValueError("stored task summary differs: {} {}".format(scene, planner))
            print("{},{},{},{},{:.2f},{:.2f},{:.2f},{:.2f}".format(
                short, planner, shape, len(group), min(joint_means), sum(joint_means),
                statistics.mean(tasks), statistics.stdev(tasks)))


if __name__ == "__main__":
    main()
