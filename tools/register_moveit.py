#!/usr/bin/env python3
"""Register the ROS-dependent planners in MoveIt 1. / 在 MoveIt 1 中注册规划器。

The four .cpp files are compiled by moveit_ompl_interface, not by libompl:
RULRRTstar and BaseRRTstar publish ROS topics and require ROS headers.
四个实现文件编入 MoveIt 接口库；规划器发布 ROS 话题并依赖 ROS 头文件。
"""

import argparse
from pathlib import Path
import re


MARKER = "# BEGIN RUL RRTSTAR RECOVERED SOURCES"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("moveit", type=Path, help="MoveIt source root, e.g. ~/ws_moveit/src/moveit")
    args = parser.parse_args()
    root = args.moveit.expanduser().resolve()
    repo = Path(__file__).resolve().parents[1]
    src = repo / "planner/src"
    context = root / "moveit_planners/ompl/ompl_interface/src/planning_context_manager.cpp"
    interface = root / "moveit_planners/ompl/ompl_interface/CMakeLists.txt"
    top = root / "moveit_planners/ompl/CMakeLists.txt"
    manifest = root / "moveit_planners/ompl/package.xml"
    for path in (context, interface, top, manifest):
        if not path.is_file():
            parser.error(f"Missing expected MoveIt 1 source file: {path}")

    ctx = context.read_text(encoding="utf-8")
    cmake = interface.read_text(encoding="utf-8")
    top_text = top.read_text(encoding="utf-8")
    manifest_text = manifest.read_text(encoding="utf-8")
    registered = all(f'"geometric::{name}"' in ctx for name in ("BaseRRTstar", "RULRRTstar"))
    installed = MARKER in cmake
    if registered != installed:
        parser.error("Found a partial or historical registration; inspect MoveIt changes before applying")
    if "/home/haibo/" in ctx or "/home/haibo/" in cmake:
        parser.error("Historical absolute paths found in MoveIt; use a clean MoveIt checkout")
    if not registered:
        include_anchor = "#include <ompl/geometric/planners/rrt/RRTstar.h>"
        if ctx.count(include_anchor) != 1:
            parser.error("RRTstar include anchor not found exactly once")
        ctx = ctx.replace(include_anchor, include_anchor + "\n"
                          "#include <ompl/geometric/planners/rrt/BaseRRTstar.h>\n"
                          "#include <ompl/geometric/planners/rrt/RULRRTstar.h>")
        pattern = re.compile(r'(  registerPlannerAllocator\([^;]+"geometric::RRTstar"[^;]+;)')
        matches = list(pattern.finditer(ctx))
        if len(matches) != 1:
            parser.error("RRTstar allocator anchor not found exactly once")
        block = """
  registerPlannerAllocator(
      "geometric::BaseRRTstar",
      std::bind(&allocatePlanner<og::BaseRRTstar>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  registerPlannerAllocator(
      "geometric::RULRRTstar",
      std::bind(&allocatePlanner<og::RULRRTstar>, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));"""
        ctx = ctx[:matches[0].end()] + block + ctx[matches[0].end():]

        sources = [
            "ompl/geometric/planners/rrt/src/BaseRRTstar.cpp",
            "ompl/geometric/planners/rrt/src/RULRRTstar.cpp",
            "ompl/base/objectives/src/BasePathLengthOptimizationObjective.cpp",
            "ompl/base/objectives/src/RULAwareOptimizationObjective.cpp",
        ]
        for relative in sources:
            if not (src / relative).is_file():
                parser.error(f"Missing recovered source: {src / relative}")
        lines = "\n".join(f'  "{(src / relative).as_posix()}"' for relative in sources)
        cmake += (f"\n{MARKER}\n"
                  f"target_sources(moveit_ompl_interface PRIVATE\n{lines}\n)\n"
                  f'target_include_directories(moveit_ompl_interface PRIVATE "{src.as_posix()}")\n'
                  "# END RUL RRTSTAR RECOVERED SOURCES\n")
        if "std_msgs" not in top_text:
            anchor = "  roscpp\n"
            if anchor not in top_text:
                parser.error("Could not add std_msgs to catkin components")
            top_text = top_text.replace(anchor, anchor + "  std_msgs\n", 1)
        if "<depend>std_msgs</depend>" not in manifest_text:
            anchor = "  <depend>roscpp</depend>"
            if manifest_text.count(anchor) != 1:
                parser.error("Could not add std_msgs to package manifest")
            manifest_text = manifest_text.replace(anchor, anchor + "\n  <depend>std_msgs</depend>")
        # Write only after all anchors and source files have passed validation.
        context.write_text(ctx, encoding="utf-8")
        interface.write_text(cmake, encoding="utf-8")
        top.write_text(top_text, encoding="utf-8")
        manifest.write_text(manifest_text, encoding="utf-8")
        print("Registered BaseRRTstar and RULRRTstar in MoveIt 1 / 已在 MoveIt 1 注册两种规划器")
    else:
        print("Planners already registered / 规划器已注册")


if __name__ == "__main__":
    main()
