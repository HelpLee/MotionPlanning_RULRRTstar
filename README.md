# RUL-aware RRT* for UR5 motion planning

**Health-aware path planning in OMPL and MoveIt 1, with a reproducible 54-run UR5 comparison.**

**UR5 寿命感知运动规划：OMPL / MoveIt 1 自定义规划器与 54 次实验数据。**

[Quick start](#quick-start--快速开始) · [Results](#results--实验结果) · [Method](docs/03_源码与算法.md) · [Reproduce](docs/01_环境与规划器注册.md) · [Data](data/joint_usage_1/) · [Citation](#citation--引用)

This repository packages `RULRRTstar`, a remaining-useful-life (RUL) aware RRT* planner, and `BaseRRTstar` for comparison. It includes the OMPL/MoveIt integration, UR5 configuration, experiment runner, and raw data for three health scenarios. The experiment plans pickup and place paths with fake execution; it does **not** execute them on a physical robot. / 本仓库提供寿命感知规划器、基线规划器、UR5 配置及三种健康场景的原始数据；实验仅规划路径，不驱动真实机器人。

![RUL-aware planning loop](docs/architecture.svg)

**Highlights / 要点**

- Custom RUL-aware and baseline RRT* planners registered in MoveIt 1 / 两种自定义规划器接入 MoveIt 1。
- Three health states × three degradation shapes × two planners × three repetitions = **54 runs** / 三种健康状态、三种退化形状、两种规划器、每组重复三次。
- Packaged CSVs, verification scripts, and reproducible summary values / 附原始 CSV、校验脚本与可复核结果。

## Results / 实验结果

In the locally degraded scenario (LDS), the mean completed tasks before the stopping condition were: / 在局部退化场景中，达到停止条件前平均完成任务数为：

| Degradation shape `p` | BaseRRTstar | RULRRTstar | Difference |
| ---: | ---: | ---: | ---: |
| 0.8 | 18.67 | 27.67 | +9.00 |
| 1.0 | 33.00 | 51.67 | +18.67 |
| 1.5 | 73.67 | 111.67 | +38.00 |

Each value is the mean of three stored runs. FHS and HDS groups reached their 80-task cap, so their task counts do not distinguish the planners. These are results of this packaged simulation setup, not evidence of physical robot lifetime extension. See the [experiment guide](docs/02_实验运行与结果.md) for settings and interpretation. / 每项为三次运行均值；FHS 和 HDS 均达到 80 个任务上限。以上是仿真实验结果，并非实体机器人寿命验证。

## Quick start / 快速开始

To inspect the published data on a Python 3 machine: / 无需 ROS 即可核对包内数据：

```bash
git clone https://github.com/HelpLee/MotionPlanning_RULRRTstar.git
cd MotionPlanning_RULRRTstar
python3 tools/check_package.py
python3 analysis/experiment_table.py --check-stored-summary
```

These two checks use the Python standard library. For the gamma trace check, install NumPy and run `python3 analysis/check_gamma_trace.py`. The full planning experiment requires **Ubuntu, ROS Melodic, MoveIt 1, OMPL 1.4.2, and a catkin workspace**; follow [environment and planner registration](docs/01_环境与规划器注册.md), then [run the experiments](docs/02_实验运行与结果.md). Third-party source revisions are in [`tools/source-revisions.json`](tools/source-revisions.json). / 前两项仅需 Python 标准库；完整规划实验需按教程配置 ROS 与 MoveIt。

## Start here / 阅读顺序

| Guide / 教程 | Scope / 内容 |
| --- | --- |
| [1. Environment and planner registration / 环境与规划器注册](docs/01_环境与规划器注册.md) | Pinned source revisions, MoveIt integration, catkin build / 源码版本、MoveIt 接线与构建 |
| [2. Experiments and results / 实验运行与结果](docs/02_实验运行与结果.md) | FHS/HDS/LDS execution, outputs, result tables / 三场景运行、输出与汇总 |
| [3. Implementation and algorithm / 源码与算法](docs/03_源码与算法.md) | Code map, data flow, comparison with standard RRT* / 代码地图、数据流与算法对比 |

## Repository layout / 仓库结构

| Path / 路径 | Contents / 内容 |
| --- | --- |
| `archive/local_history_20250930/` | Unmodified historical Python/C++ source snapshots and SHA-256 manifest / 原始源码快照与校验清单 |
| `planner/src/ompl/` | Two planners and two optimization objectives, each with headers and implementations / 两个规划器和两个优化目标的头文件与实现 |
| `tools/` | Source revision manifest, MoveIt registration, package validation / 依赖版本、注册工具与包检查 |
| `ros/ur5_moveit_config/` | UR5 MoveIt configuration, including custom planner IDs and fake execution / UR5 规划配置与仿真启动文件 |
| `experiment/run_experiment.py` | Portable three-scenario batch runner / 三场景批量运行脚本 |
| `data/joint_usage_1/` | 54 runs, paired usage/loss files, and saved summaries / 原始运行数据及汇总 |
| `analysis/` | Data checks, summary table, and plotting utility / 数据检查、统计表与绘图工具 |

## Experiment design / 实验设计

| Scenario / 场景 | Initial joint RUL / 六关节初始 RUL | Task limit / 任务上限 |
| --- | --- | ---: |
| FHS — Full Health / 全健康 | `[1000,1000,1000,1000,1000,1000]` | 80 |
| HDS — Heterogeneous Degradation / 非均匀退化 | `[500,800,400,600,1000,700]` | 80 |
| LDS — Local Degradation / 局部退化 | `[100,1000,1000,1000,1000,1000]` | 500; stop at `min_rul=10` / 达阈值停止 |

Each scenario uses degradation shape `p ∈ {0.8, 1.0, 1.5}`. Each shape is run three times with `RULRRTstar` and three times with `BaseRRTstar`, for **54 runs**. The runner uses `alpha=lambda=rulmin=1`, a five-second planning limit, and one planning attempt per request. `EPSILON_INIT=1e-9` is rounded to `0.0` when the 16-column parameter CSV is written. / 每个场景有三种退化形状，每种形状下两个规划器各运行三次，共 **54 次**。其余参数及 CSV 舍入规则见[实验教程](docs/02_实验运行与结果.md)。

The experiment uses UR5 fake execution. It plans pickup and place paths and derives joint usage from those paths; `move_group.execute()` remains commented in the original experiment. / 实验采用 UR5 仿真执行配置，对取放路径进行规划并据此计算关节用量；原实验脚本未调用真实机器人执行接口。

## Inspect the packaged results / 检查包内数据

From the repository root / 在仓库根目录运行：

```bash
python3 tools/check_package.py
python3 analysis/experiment_table.py --check-stored-summary
python3 analysis/check_gamma_trace.py  # requires NumPy / 需要 NumPy
```

The first command checks source hashes and dataset completeness; the second produces 18 planner/shape/scenario summary rows; the third checks the initial gamma values in all 54 runs. / 三条命令分别检查源码与数据完整性、输出 18 组统计结果、核对 54 次运行的初始 gamma。

## Integration model / 集成方式

The planners implement OMPL APIs but also publish ROS topics. Their four `.cpp` files are therefore compiled into MoveIt 1's `moveit_ompl_interface`. `planning_context_manager.cpp` registers `geometric::RULRRTstar` and `geometric::BaseRRTstar`; `ompl_planning.yaml` exposes both IDs to the UR5 `manipulator` group. / 规划器遵循 OMPL 接口，同时发布 ROS 话题，因此源码编入 MoveIt 的 OMPL 接口库，由 MoveIt 注册规划器 ID，再在 UR5 的 YAML 中提供给 `manipulator` 规划组。

The project targets the ROS Melodic / MoveIt 1 / OMPL 1.4.2 API generation. Third-party source commits are listed in `tools/source-revisions.json`. For API background, see the official [OMPL planner guide](https://ompl.kavrakilab.org/newPlanner.html) and [MoveIt OMPL configuration guide](https://moveit.github.io/moveit_tutorials/doc/ompl_interface/ompl_interface_tutorial.html). / 第三方源码版本已在清单中固定；上述官方文档可用于查阅接口约定。

## Citation / 引用

If you use this repository, cite the software using [`CITATION.cff`](CITATION.cff). A paper link and formal publication citation will be added when available. / 使用本仓库时请引用软件；论文正式信息将在公开后补充。

## License / 许可

No project-wide software license has been declared yet. The repository also references third-party ROS, MoveIt, and OMPL sources; inspect their licenses before reuse or redistribution. / 目前尚未声明覆盖整个仓库的软件许可；复用或再分发前请检查第三方组件的许可。
