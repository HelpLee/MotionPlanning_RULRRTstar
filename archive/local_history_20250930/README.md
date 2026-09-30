# Original source snapshot / 原始源码快照

此目录保存 2025 年三场景实验的原始 `ur5_group.py`、`RULRRTstar.cpp`、`RULAwareOptimizationObjective.cpp` 和原始 `SHA256SUMS.txt`。三个源码文件保持原字节内容，供对照算法和参数。`SHA256SUMS.relative.txt` 用仓库内相对路径保存相同哈希。

This directory preserves the original `ur5_group.py`, `RULRRTstar.cpp`, `RULAwareOptimizationObjective.cpp`, and `SHA256SUMS.txt` for the three-scenario experiment. The three source files remain byte-for-byte unchanged. `SHA256SUMS.relative.txt` records the same hashes with repository-relative paths.

```bash
sha256sum -c SHA256SUMS.relative.txt
```

请在 `experiment/run_experiment.py` 和 `planner/src/ompl/` 的工作副本上修改、构建和运行。仓库根目录的 [README](../../README.md) 提供完整操作路径。

Edit and run the working sources in `experiment/run_experiment.py` and `planner/src/ompl/`. The repository [README](../../README.md) links the complete workflow.
