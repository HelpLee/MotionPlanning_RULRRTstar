#!/usr/bin/env python3
"""Plot tasks and weakest-joint RUL. / 绘制任务数与最弱关节 RUL。"""

import argparse
import statistics
from pathlib import Path

from experiment_table import ROOT, RUN, SCENES, read_run


def collect():
    result = {}
    for folder_name, scene in SCENES.items():
        for path in (ROOT / folder_name).glob("*.csv"):
            match = RUN.fullmatch(path.name)
            if match:
                final, tasks, _, _ = read_run(path)
                result.setdefault((scene, match.group(1), match.group(2)), []).append((final, tasks))
    if len(result) != 18 or any(len(group) != 3 for group in result.values()):
        raise ValueError("Expected 18 groups with three runs each")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("results/summary.png"))
    args = parser.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    groups = collect()
    shapes = ("0.8", "1", "1.5")
    scenes = ("FHS", "HDS", "LDS")
    planners = ("base", "rul-aware")
    colors = {"base": "#7c8ca3", "rul-aware": "#117a65"}
    fig, axes = plt.subplots(2, 3, figsize=(13, 7))
    for column, scene in enumerate(scenes):
        for row, metric in enumerate(("tasks", "weakest final RUL")):
            ax = axes[row, column]
            for planner_index, planner in enumerate(planners):
                values = []
                for shape in shapes:
                    runs = groups[(scene, planner, shape)]
                    if row == 0:
                        values.append(statistics.mean(task for _, task in runs))
                    else:
                        mean_by_joint = [statistics.mean(final[j] for final, _ in runs) for j in range(6)]
                        values.append(min(mean_by_joint))
                x = [i + (-0.18 if planner_index == 0 else 0.18) for i in range(3)]
                ax.bar(x, values, width=0.34, color=colors[planner], label=planner)
            ax.set_xticks(range(3))
            ax.set_xticklabels(shapes)
            ax.set_xlabel("degradation shape p")
            ax.set_ylabel(metric)
            ax.set_title(scene)
            ax.grid(axis="y", alpha=0.2)
            if row == 0 and column == 0:
                ax.legend()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(args.out, dpi=180)
    print(args.out.resolve())


if __name__ == "__main__":
    main()
