#!/usr/bin/env python3
"""Check gamma against 54 initial rows. / 核对 54 次运行的初始 gamma。"""

import csv
import importlib.util
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "analysis/cala_gamma.py"
DATA = ROOT / "data/joint_usage_1"
RUN = re.compile(r"^(base|rul-aware)_(0\.8|1|1\.5)_[123]\.csv$")


def main():
    spec = importlib.util.spec_from_file_location("historical_gamma", SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    checked = 0
    for scene in ("Full_Health_System", "Heterogeneous_Degradation_System", "Local_Degradation_System"):
        for path in (DATA / scene).glob("*.csv"):
            if not RUN.fullmatch(path.name):
                continue
            with path.open(newline="", encoding="utf-8") as stream:
                row = next(csv.reader(stream))
            initial_rul = [float(value) for value in row[:6]]
            saved_gamma = [float(value) for value in row[6:12]]
            computed_gamma = module.compute_gamma_from_rul_enhanced(initial_rul)[2]
            if any(abs(round(float(value), 2) - saved) > 0.001
                   for value, saved in zip(computed_gamma, saved_gamma)):
                raise ValueError("initial gamma differs: {}".format(path))
            checked += 1
    if checked != 54:
        raise ValueError("expected 54 runs, found {}".format(checked))
    print("Gamma matches all {} initial rows / 初始 gamma 与全部 {} 次运行一致。".format(checked, checked))


if __name__ == "__main__":
    main()
