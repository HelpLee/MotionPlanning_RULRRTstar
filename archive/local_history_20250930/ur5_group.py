#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
UR5 多次 pick-place 演示（批量实验三阶段版，保持核心逻辑不变）
阶段A：Local_Degradation_System
  - STARTING_RUL = [100, 1000, 1000, 1000, 1000, 1000], max_tasks=50
阶段B：Full_Health_System
  - STARTING_RUL = [1000, 1000, 1000, 1000, 1000, 1000], max_tasks=50
阶段C：Heterogeneous_Degradation_System
  - STARTING_RUL = [400, 800, 300, 500, 1000, 600],   max_tasks=30

每阶段内：
  - RULRRTstar 与 BaseRRTstar 各自运行
  - RUL_SHAPE_P ∈ {0.8, 1.0, 1.5}，每个做 5 次
  - 每次结束重命名/移动：rul.csv → {prefix}_{shape}_{idx}.csv
                         joint_usage_*.csv → {prefix}_{shape}_{idx}_joint_usage.csv
                         joint_loss_*.csv  → {prefix}_{shape}_{idx}_joint_loss.csv
  其他算法与运行逻辑完全不改
"""

from __future__ import print_function
import os, sys, csv, time, math, rospy, random, select, threading, datetime, copy, shutil
import numpy as np
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker

# ====================== 批量实验全局配置 ======================
# 形状与重复次数
SHAPE_LIST = [0.8, 1.0, 1.5]
# SHAPE_LIST = [0.8]  # 调试时用
RUNS_PER_SHAPE = 3

# 三个阶段输出目录（可按需修改）
EXPORT_DIR_STAGE_A = "/home/haibo/catkin_ws/src/pick_place_demo/src/joint_usage/Local_Degradation_System"
EXPORT_DIR_STAGE_B = "/home/haibo/catkin_ws/src/pick_place_demo/src/joint_usage/Full_Health_System"
EXPORT_DIR_STAGE_C = "/home/haibo/catkin_ws/src/pick_place_demo/src/joint_usage/Heterogeneous_Degradation_System"
# ============================================================

# ====================== 常量配置（原逻辑保持） ======================
SEED = 42
USAGE_TIMEOUT = 1.0

WS_X = (0.15, 0.55)
WS_Y_Pick = (-0.50, -0.35)
WS_Y_Place = (0.40, 0.55)
WS_Z = (0.10, 0.20)

NUM_JOINTS = 6
RUL_INIT   = [1000.0] * NUM_JOINTS

GAMMA_INIT = [1.0] * NUM_JOINTS
ALPHA_INIT = 1.0
LAMBDA_INIT = 1.0
RULMIN_PARAM = 1.0
EPSILON_INIT = 1e-9

# 会在每个阶段前被覆盖
STARTING_RUL = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]

RUL_FLOOR   = [10.0] * NUM_JOINTS
RUL_SHAPE_P = [0.8] * NUM_JOINTS       # 会在批量实验里按需覆盖
XMAX_PER_JOINT = [1000] * NUM_JOINTS

TIMESTAMP = None  # 会在每次实验前刷新
PKG_DATA_DIR = os.path.expanduser("/home/haibo/catkin_ws/src/pick_place_demo/src/joint_usage")
CSV_PATH_USAGE = None  # 每次实验前刷新
CSV_PATH_LOSS  = None  # 每次实验前刷新
RUL_CSV_PATH   = os.path.join(PKG_DATA_DIR, "rul.csv")  # 固定名，实验后会重命名/移动

PLANNING_GROUP = "manipulator"
PLANNER_ID = "RULRRTstar"             # 会在批量实验里按需切换
PLANNING_TIME = 5.0
PLANNING_ATTEMPTS = 1

# 会在每个阶段前被覆盖
max_tasks = 50
min_rul   = 10

HOME_JOINTS = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]

# 当前阶段的导出目录（每阶段开始时覆盖）
EXPORT_DIR = EXPORT_DIR_STAGE_A

# ====================== 全局变量（原逻辑保持） ======================
def _as_list(val, n, name):
    if isinstance(val, (list, tuple)):
        if len(val) != n:
            raise ValueError("{} length {} != expected {}".format(name, len(val), n))
        return [float(x) for x in val]
    return [float(val)] * n

_current_rul    = _as_list(RUL_INIT, NUM_JOINTS, "RUL_INIT")
_current_gamma  = _as_list(GAMMA_INIT, NUM_JOINTS, "GAMMA_INIT")
_current_alpha  = float(ALPHA_INIT)
_current_lambda = float(LAMBDA_INIT)
_current_rulmin = float(RULMIN_PARAM)
_current_eps    = float(EPSILON_INIT)

_prev_usage = None
_rul_file_ready = False
EXIT_REQUESTED  = False
marker_pub      = None
move_group      = None
_cum_usage = [0.0] * NUM_JOINTS

# ====================== 小工具：文件与快照（原逻辑保持） ======================
def _ensure_dir(path):
    d = os.path.dirname(path)
    if d and not os.path.exists(d):
        os.makedirs(d)

def _file_size(path):
    try:
        return os.path.getsize(path)
    except OSError:
        return 0

def _safe_truncate(path, size_bytes):
    _ensure_dir(path)
    with open(path, "a+b") as f:
        f.flush()
        os.fsync(f.fileno())
    with open(path, "r+b") as f:
        f.truncate(size_bytes)
        f.flush()
        os.fsync(f.fileno())

# ====================== gamma 参数更新（原逻辑保持） ======================
# def _update_gamma_from_rul_cv(rho=0.0, eps=1e-9, tiny=1.0, p_min=4, alpha=1.5, p_max=None):
#     global _current_gamma
#     _current_gamma = [0.2, 0.2, 2.0, 0.2, 0.2, 0.2]

# def _update_gamma_from_rul_cv(rho=0.0, eps=1e-9, tiny=1.0, p_min=4, alpha=1.5, p_max=None):
#     """
#     Python2 版本：p = max(p_min, 1 + alpha * B)，可选再加上限 p_max
#     """
#     global _current_gamma
#     R = np.asarray(_current_rul, dtype=float)
#     J = len(R)

#     mean = max(float(R.mean()), eps)
#     std  = float(R.std())
#     B = std / mean

#     # 力度参数 p（保证最小 p_min）
#     p_raw = 1.0 + alpha * B
#     p = max(p_min, p_raw)
#     if p_max is not None:
#         p = min(p, p_max)

#     # 权重与归一化（mean(gamma)=1）
#     w = (1.0 / np.maximum(R, tiny)) ** p
#     s = max(float(w.sum()), eps)
#     gamma_target = (J * w / s).astype(float)

#     # EMA 平滑（可选）
#     if rho > 0.0:
#         _current_gamma = (1 - rho) * np.asarray(_current_gamma, float) + rho * gamma_target
#         _current_gamma = _current_gamma.tolist()
#     else:
#         _current_gamma = gamma_target.tolist()


def _update_gamma_from_rul_cv(
    rho=0.0, eps=1e-9, tiny=1e-6, p_max=1, c=0.0001, k=3,
    # === 新增：放大逻辑的总开关与参数 ===
    boost=True,           # False =关闭增强（保持原逻辑）； True =开启增强
    r_target=2.0,          # 目标对比度：想要 γ_minRUL : γ_maxRUL ≈ r_target : 1
    p_cap=10000.0,            # p 的上限，防极端
    tau=1.8,               # 轻度幂放大（sharpen）强度，1.5~2.0 较稳
    gmin=0.10, gmax=2.50,  # γ 的上下限，防抖动与极端
    debug=False            # 打印调试信息
):
    """
    稳健版 gamma 更新（可选增强）：
    1) RUL 归一化到 [0,1]，计算 B=std/mean
    2) p(B) = 1 + p_max * B^k / (B^k + c^k)
    3) (可选) 目标对比度反推 p、轻度 sharpen、上下限
    4) EMA 平滑（rho）
    """
    global _current_gamma

    R = np.asarray(_current_rul, dtype=float)
    J = len(R)

    # --- 异常检测 ---
    if R.size == 0 or np.any(R <= 0) or np.any(~np.isfinite(R)):
        try:
            rospy.logwarn("RUL 数据异常，gamma 重置为 1")
        except Exception:
            pass
        _current_gamma = np.ones(J, dtype=float).tolist()
        return

    # --- 归一化 RUL 到 [0,1] ---
    R_norm = R / (np.max(R) + eps)
    R_norm = np.clip(R_norm, 1e-3, 1.0)

    # --- 计算 B ---
    mean_R = np.mean(R_norm)
    std_R = np.std(R_norm)
    B = std_R / (mean_R + eps)

    # --- 基础 p(B) ---
    p = 1.0 + p_max * (B ** k) / (B ** k + c ** k + 1e-9)

    # === 增强分支（可开关） ===
    if boost:
        # 1) 用目标对比度反推 p，并与 p(B) 取较大者
        ratio = float(np.max(R) / (np.min(R) + eps))  # 用原始 R，避免归一化剪裁影响
        if ratio > 1.0 + 1e-12 and r_target is not None:
            p_gap = float(np.log(r_target) / np.log(ratio))
            p = float(np.clip(max(p, p_gap), 1.0, p_cap))

    # --- 基础权重与归一化（均值=1） ---
    w = (1.0 / np.maximum(R_norm, tiny)) ** p
    w = np.clip(w, 1e-12, 1e12)
    s = np.sum(w)
    if s < 1e-12:
        try:
            rospy.logwarn("权重求和过小，gamma 重置为 1")
        except Exception:
            pass
        gamma_target = np.ones(J, dtype=float)
    else:
        gamma_target = (J * w / s).astype(float)

    # --- 增强的后处理（可开关） ---
    if boost:
        # 2) 轻度 sharpen（幂放大）+ 重新归一化（均值=1）
        if tau is not None and tau > 1.0:
            g2 = gamma_target ** tau
            gamma_target = J * g2 / (np.sum(g2) + eps)

        # 3) 上下限裁剪 + 重新归一化（均值=1）
        if gmin is not None and gmax is not None and gmax > gmin:
            gamma_target = np.clip(gamma_target, gmin, gmax)
            gamma_target = J * gamma_target / (np.sum(gamma_target) + eps)

    # --- EMA 平滑（rho 为新值占比） ---
    if rho > 0.0 and _current_gamma is not None and len(_current_gamma) == J:
        old = np.asarray(_current_gamma, dtype=float)
        if np.all(np.isfinite(gamma_target)):
            gamma = (1 - rho) * old + rho * gamma_target
            # 保持均值=1
            gamma = J * gamma / (np.sum(gamma) + eps)
        else:
            gamma = old
    else:
        gamma = gamma_target

    _current_gamma = gamma.tolist()

    if debug:
        try:
            rospy.loginfo("[gamma] B=%.5f  p=%.3f  boost=%s  rho=%.2f  "
                          "min=%.3f max=%.3f ratio=%.2f",
                          B, p, str(boost), rho,
                          float(np.min(gamma)), float(np.max(gamma)),
                          float(np.max(gamma)/(np.min(gamma)+eps)))
        except Exception:
            print("[gamma] B=%.5f p=%.3f boost=%s rho=%.2f min=%.3f max=%.3f ratio=%.2f"
                  % (B, p, str(boost), rho,
                     float(np.min(gamma)), float(np.max(gamma)),
                     float(np.max(gamma)/(np.min(gamma)+eps))))



# ====================== RUL CSV（原逻辑保持） ======================
def _pack_param_row():
    fmt = lambda x: round(float(x), 2)
    return [fmt(x) for x in _current_rul] + \
           [fmt(x) for x in _current_gamma] + \
           [fmt(_current_lambda), fmt(_current_alpha), fmt(_current_rulmin), fmt(_current_eps)]


def _ensure_rul_csv_initialized():
    global _rul_file_ready
    _ensure_dir(RUL_CSV_PATH)
    if (not os.path.exists(RUL_CSV_PATH)) or _file_size(RUL_CSV_PATH) == 0:
        with open(RUL_CSV_PATH, "ab") as f:
            csv.writer(f).writerow(_pack_param_row())
            f.flush()
            os.fsync(f.fileno())
    _rul_file_ready = True

def _append_rul_row():
    with open(RUL_CSV_PATH, "ab") as f:
        w = csv.writer(f)
        w.writerow(_pack_param_row())
        f.flush()
        os.fsync(f.fileno())
    rospy.loginfo("RUL appended: {}".format([round(x, 2) for x in _current_rul]))

def ensure_csv_header(path, first_row_len):
    _ensure_dir(path)
    if (not os.path.exists(path)) or _file_size(path) == 0:
        with open(path, "wb") as f:
            csv.writer(f).writerow(["task", "phase"] + ["v{}".format(i) for i in range(first_row_len)])

def append_csv_rows(path, rows):
    if not rows:
        return
    with open(path, "ab") as f:
        w = csv.writer(f)
        for r in rows:
            r_out = [r[0], r[1]] + [round(float(x), 2) for x in r[2:]]
            w.writerow(r_out)
        f.flush()
        os.fsync(f.fileno())


# ====================== RUL 更新（原逻辑保持） ======================
def _normalize_usage_vec(raw):
    if raw is None:
        return None
    v = list(raw)
    if len(v) >= NUM_JOINTS + 1:
        v = v[1:1+NUM_JOINTS]
    elif len(v) != NUM_JOINTS:
        v = (v + [0.0] * NUM_JOINTS)[:NUM_JOINTS]
    return [float(x) for x in v]

def _compute_delta_from_usage(usage_now, prev_usage):
    if usage_now is None:
        return None
    if prev_usage is None:
        return [max(u, 0.0) for u in usage_now]
    reset_like = any(usage_now[j] < prev_usage[j]
                     for j in range(min(len(usage_now), len(prev_usage))))
    delta = []
    for j in range(NUM_JOINTS):
        u = usage_now[j] if j < len(usage_now) else 0.0
        p = prev_usage[j] if j < len(prev_usage) else 0.0
        d = max(u, 0.0) if reset_like else max(u - p, 0.0)
        delta.append(d)
    return delta

def _rul_curve(x, R0, Rmin, xmax, p):
    if xmax <= 0:
        return max(Rmin, min(R0, R0))
    if x <= 0:
        return R0
    if x >= xmax:
        return Rmin - (x - xmax)
    t = x / float(xmax)
    return Rmin + (R0 - Rmin) * pow(1.0 - t, p)

def bootstrap_used_motor_rul(starting_rul):
    global _cum_usage, _current_rul
    _current_rul = [float(x) for x in starting_rul]
    _cum_usage = []
    for j in range(NUM_JOINTS):
        R0   = float(RUL_INIT[j])
        Rcur = float(starting_rul[j])
        Rmin = float(RUL_FLOOR[j])
        xmax = float(XMAX_PER_JOINT[j])
        p    = float(RUL_SHAPE_P[j])
        s = 0.0 if R0 <= Rmin else max(0.0, min(1.0, (Rcur - Rmin) / (R0 - Rmin)))
        t = s ** (1.0 / p)
        xcur = (1.0 - t) * xmax
        _cum_usage.append(xcur)

def update_rul_unified():
    global _current_rul, _prev_usage, _cum_usage
    raw = get_usage_once(timeout_sec=USAGE_TIMEOUT)
    if raw is None:
        if _prev_usage is None:
            rospy.logwarn("No /joint_usage on first call; skip RUL update this time.")
            return
        usage = _prev_usage
        rospy.logwarn("No /joint_usage this time; using previous usage vector.")
    else:
        usage = _normalize_usage_vec(raw)

    delta = _compute_delta_from_usage(usage, _prev_usage)
    _prev_usage = list(usage)
    if delta is None:
        rospy.logwarn("Delta usage is None; skip RUL update.")
        return

    for j in range(NUM_JOINTS):
        _cum_usage[j] += max(0.0, float(delta[j]))

    for j in range(NUM_JOINTS):
        _current_rul[j] = _rul_curve(
            x=_cum_usage[j],
            R0=RUL_INIT[j],
            Rmin=RUL_FLOOR[j],
            xmax=XMAX_PER_JOINT[j],
            p=RUL_SHAPE_P[j]
        )

    _update_gamma_from_rul_cv()
    _append_rul_row()

# ====================== ROS 消息读取（原逻辑保持） ======================
def get_usage_once(timeout_sec=USAGE_TIMEOUT):
    try:
        msg = rospy.wait_for_message("/joint_usage", Float64MultiArray, timeout=timeout_sec)
        return list(msg.data)
    except rospy.ROSException:
        return None

def get_loss_once(timeout_sec=USAGE_TIMEOUT):
    try:
        msg = rospy.wait_for_message("/joint_loss", Float64MultiArray, timeout=timeout_sec)
        return list(msg.data)
    except rospy.ROSException:
        return None

# ====================== 退出监听（原逻辑保持） ======================
def start_exit_listener(move_group_obj):
    def _listener():
        global EXIT_REQUESTED
        rospy.loginfo("Press 'q' + Enter to stop immediately.")
        while not rospy.is_shutdown() and not EXIT_REQUESTED:
            try:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    if sys.stdin.readline().strip().lower() == 'q':
                        EXIT_REQUESTED = True
                        rospy.logwarn("Quit signal received; stopping the robot.")
                        try:
                            move_group_obj.stop()
                        except:
                            pass
                        rospy.signal_shutdown("User requested quit")
                        break
            except:
                pass
    th = threading.Thread(target=_listener)
    th.daemon = True
    th.start()

# ====================== 标记与采样（原逻辑保持） ======================
def create_marker(pose, marker_id, color=(0.0, 1.0, 0.0)):
    m = Marker()
    m.header.frame_id = move_group.get_planning_frame()
    m.header.stamp = rospy.Time.now()
    m.ns = "pick_place_markers"
    m.id = marker_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose = pose
    m.scale.x = m.scale.y = m.scale.z = 0.05
    m.color.r, m.color.g, m.color.b = color
    m.color.a = 1.0
    m.lifetime = rospy.Duration(0)
    return m

def create_text_marker(text, pose, marker_id, color=(1.0, 1.0, 1.0)):
    m = Marker()
    m.header.frame_id = move_group.get_planning_frame()
    m.header.stamp = rospy.Time.now()
    m.ns = "pick_place_text"
    m.id = marker_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = pose.position.x
    m.pose.position.y = pose.position.y
    m.pose.position.z = pose.position.z + 0.15
    m.pose.orientation.w = 1.0
    m.scale.z = 0.1
    m.color.r, m.color.g, m.color.b = color
    m.color.a = 1.0
    m.text = text
    m.lifetime = rospy.Duration(0)
    return m

def sample_pickup_pose():
    p = geometry_msgs.msg.Pose()
    p.position.x = np.random.uniform(*WS_X)
    p.position.y = np.random.uniform(*WS_Y_Pick)
    p.position.z = np.random.uniform(*WS_Z)
    p.orientation.w = 1.0
    return p

def sample_place_pose():
    p = geometry_msgs.msg.Pose()
    p.position.x = np.random.uniform(*WS_X)
    p.position.y = np.random.uniform(*WS_Y_Place)
    p.position.z = np.random.uniform(*WS_Z)
    p.orientation.w = 1.0
    return p

# ====================== 事务（原逻辑保持） ======================
class TaskTransaction(object):
    def __init__(self, rul_csv_path):
        self.rul_csv_path = rul_csv_path
        self.file_size_before = _file_size(rul_csv_path)
        self.snap_rul = copy.deepcopy(_current_rul)
        self.snap_gamma = copy.deepcopy(_current_gamma)
        self.snap_alpha = _current_alpha
        self.snap_lambda = _current_lambda
        self.snap_rulmin = _current_rulmin
        self.snap_eps = _current_eps
        self.snap_prev_usage = copy.deepcopy(_prev_usage)

    def rollback(self):
        global _current_rul, _current_gamma, _current_alpha, _current_lambda, _current_rulmin, _current_eps, _prev_usage
        _current_rul = copy.deepcopy(self.snap_rul)
        _current_gamma = copy.deepcopy(self.snap_gamma)
        _current_alpha = self.snap_alpha
        _current_lambda = self.snap_lambda
        _current_rulmin = self.snap_rulmin
        _current_eps = self.snap_eps
        _prev_usage = copy.deepcopy(self.snap_prev_usage)
        _safe_truncate(self.rul_csv_path, self.file_size_before)
        rospy.logwarn("Rollback: RUL state and rul.csv restored to the state before the task.")

    def commit(self):
        rospy.loginfo("Commit: task kept; RUL changes retained.")

# ====================== MoveIt 运动（原逻辑保持） ======================
def move_to_pose(pose, step_name):
    if EXIT_REQUESTED or rospy.is_shutdown():
        return False
    rospy.loginfo("Planning to {}".format(step_name))
    move_group.set_pose_target(pose)
    plan = move_group.plan()
    if EXIT_REQUESTED or rospy.is_shutdown():
        move_group.clear_pose_targets()
        return False
    if not plan or len(plan.joint_trajectory.points) == 0:
        rospy.logwarn("Planning failed for {}".format(step_name))
        move_group.clear_pose_targets()
        return False
    rospy.loginfo("Planning succeeded for {}".format(step_name))
    # exec_ok = move_group.execute(plan, wait=True)
    exec_ok = True
    move_group.stop()
    move_group.clear_pose_targets()
    if exec_ok:
        update_rul_unified()
    if not exec_ok or EXIT_REQUESTED or rospy.is_shutdown():
        rospy.logwarn("Execution failed for {}".format(step_name))
        return False
    rospy.loginfo("Execution finished for {}".format(step_name))
    return True

def move_to_home(update_rul=True):
    if EXIT_REQUESTED or rospy.is_shutdown():
        return False
    rospy.loginfo("Planning to Home")
    move_group.set_joint_value_target(HOME_JOINTS)
    plan = move_group.plan()
    if EXIT_REQUESTED or rospy.is_shutdown():
        return False
    if not plan or len(plan.joint_trajectory.points) == 0:
        rospy.logwarn("Planning failed for Home")
        return False
    rospy.loginfo("Planning succeeded for Home")
    # exec_ok = move_group.execute(plan, wait=True)
    exec_ok = True
    move_group.stop()
    if exec_ok and update_rul:
        update_rul_unified()
    if not exec_ok or EXIT_REQUESTED or rospy.is_shutdown():
        rospy.logwarn("Execution failed for Home")
        return False
    rospy.loginfo("Execution finished for Home")
    return True

# ====================== 停机条件（原逻辑保持） ======================
def should_stop(success_count, current_rul, max_tasks=None, min_rul=None):
    if max_tasks is not None and success_count >= max_tasks:
        rospy.logwarn("Max tasks reached: {}".format(max_tasks))
        return True
    if min_rul is not None and min(current_rul) <= min_rul:
        rospy.logwarn("RUL <= {:.1f}, stopping after this task".format(min_rul))
        return True
    return False

# ====================== —— 批量实验外壳，不改核心逻辑 —— ======================
def _reset_globals_for_run():
    global _current_rul, _current_gamma, _current_alpha, _current_lambda, _current_rulmin, _current_eps
    global _prev_usage, _rul_file_ready, EXIT_REQUESTED, _cum_usage
    _current_rul    = _as_list(RUL_INIT, NUM_JOINTS, "RUL_INIT")
    _current_gamma  = _as_list(GAMMA_INIT, NUM_JOINTS, "GAMMA_INIT")
    _current_alpha  = float(ALPHA_INIT)
    _current_lambda = float(LAMBDA_INIT)
    _current_rulmin = float(RULMIN_PARAM)
    _current_eps    = float(EPSILON_INIT)
    _prev_usage = None
    _rul_file_ready = False
    EXIT_REQUESTED = False
    _cum_usage = [0.0] * NUM_JOINTS

def _set_paths_for_new_run():
    global TIMESTAMP, CSV_PATH_USAGE, CSV_PATH_LOSS
    TIMESTAMP = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    CSV_PATH_USAGE = os.path.join(PKG_DATA_DIR, "joint_usage_{}.csv".format(TIMESTAMP))
    CSV_PATH_LOSS  = os.path.join(PKG_DATA_DIR, "joint_loss_{}.csv".format(TIMESTAMP))

def _prepare_export_dir(dir_path):
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)

def _move_outputs(prefix, shape_val, run_idx, export_dir):
    """把本轮实验输出重命名并移动到 export_dir"""
    _prepare_export_dir(export_dir)
    # 目标文件名
    param_str = str(shape_val).rstrip('0').rstrip('.') if isinstance(shape_val, float) else str(shape_val)
    rul_new   = "{}_{}_{}.csv".format(prefix, param_str, run_idx)
    usage_new = "{}_{}_{}_joint_usage.csv".format(prefix, param_str, run_idx)
    loss_new  = "{}_{}_{}_joint_loss.csv".format(prefix, param_str, run_idx)

    # 源文件
    src_files = [
        (RUL_CSV_PATH, rul_new),
        (CSV_PATH_USAGE, usage_new),
        (CSV_PATH_LOSS,  loss_new),
    ]
    for src, dst_name in src_files:
        if src and os.path.exists(src):
            dst = os.path.join(export_dir, dst_name)
            try:
                shutil.move(src, dst)
                rospy.loginfo("Moved: {} -> {}".format(src, dst))
            except Exception as e:
                rospy.logwarn("Move failed {} -> {}: {}".format(src, dst, e))
        else:
            rospy.logwarn("File not found, skip moving: {}".format(src))

def _run_one_experiment():
    """一轮实验（原主循环逻辑保持不变）"""
    try:
        rospy.loginfo("Pre-homing before all tasks")
        move_to_home(update_rul=False)
        rospy.sleep(0.2)

        first_header_usage = False
        first_header_loss  = False
        rate = rospy.Rate(10)

        success_count = 0
        attempt_count = 0
        task_index    = 0

        while not EXIT_REQUESTED and not rospy.is_shutdown():
            attempt_count += 1
            task_index = success_count + 1
            rospy.loginfo("========= New Task (target #{}) =========".format(task_index))

            pickup_pose = sample_pickup_pose()
            place_pose  = sample_place_pose()

            rospy.loginfo("Task #{} Pickup Pose: x={:.3f}, y={:.3f}, z={:.3f}".format(
                task_index, pickup_pose.position.x, pickup_pose.position.y, pickup_pose.position.z))
            rospy.loginfo("Task #{} Place  Pose: x={:.3f}, y={:.3f}, z={:.3f}".format(
                task_index, place_pose.position.x, place_pose.position.y, place_pose.position.z))
            rospy.loginfo("Task #{} Home Joints: {}".format(task_index, [round(v, 3) for v in HOME_JOINTS]))

            base_id = 100 + attempt_count * 10
            for m in [
                create_marker(pickup_pose, base_id + 1, (0, 0, 1)),
                create_marker(place_pose,  base_id + 2, (1, 0, 0)),
                create_text_marker("Pickup target #{}".format(task_index), pickup_pose, base_id + 3),
                create_text_marker("Place  target #{}".format(task_index), place_pose,  base_id + 4),
            ]:
                marker_pub.publish(m)
                rate.sleep()

            while not EXIT_REQUESTED and not rospy.is_shutdown():
                txn = TaskTransaction(RUL_CSV_PATH)

                usage_rows, loss_rows = [], []
                ok = True

                if not move_to_pose(pickup_pose, "Pickup #{}".format(task_index)):
                    ok = False
                else:
                    usage = get_usage_once()
                    loss  = get_loss_once()
                    if usage is None or loss is None:
                        ok = False
                    else:
                        if not first_header_usage:
                            ensure_csv_header(CSV_PATH_USAGE, len(usage)); first_header_usage = True
                        if not first_header_loss:
                            ensure_csv_header(CSV_PATH_LOSS, len(loss));   first_header_loss  = True
                        usage_rows.append([task_index, "pickup"] + usage)
                        loss_rows.append( [task_index, "pickup"] + loss)

                if ok and move_to_pose(place_pose, "Place #{}".format(task_index)):
                    usage = get_usage_once()
                    loss  = get_loss_once()
                    if usage and loss:
                        usage_rows.append([task_index, "place"] + usage)
                        loss_rows.append( [task_index, "place"] + loss)
                    else:
                        ok = False
                else:
                    ok = False

                # if ok and move_to_home(update_rul=True):
                #     usage = get_usage_once()
                #     loss  = get_loss_once()
                #     if usage and loss:
                #         usage_rows.append([task_index, "home"] + usage)
                #         loss_rows.append( [task_index, "home"] + loss)
                #     else:
                #         ok = False
                # else:
                #     ok = False

                if ok:
                    txn.commit()
                    append_csv_rows(CSV_PATH_USAGE, usage_rows)
                    append_csv_rows(CSV_PATH_LOSS,  loss_rows)
                    success_count += 1
                    rospy.loginfo("Task #{} completed. (total_success={})".format(task_index, success_count))
                    break
                else:
                    rospy.logwarn("Task #{} failed, rolling back and retrying same poses ...".format(task_index))
                    txn.rollback()
                    # _ = move_to_home(update_rul=False)
                    txn.rollback()
                    try:
                        move_group.stop()
                        move_group.clear_pose_targets()
                    except Exception:
                        pass
                    rospy.sleep(0.1)
                    rospy.sleep(0.1)

            if should_stop(success_count, _current_rul, max_tasks=max_tasks, min_rul=min_rul):
                break

        rospy.loginfo("Experiment finished: successful_tasks={}, total_attempts={}".format(success_count, attempt_count))

    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException caught. Exiting.")
    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupt detected. Exiting.")
    except Exception as e:
        rospy.logwarn(str(e))

def _run_batch(planner_id, prefix, export_dir):
    """按给定 planner 与前缀，依次跑 SHAPE_LIST × RUNS_PER_SHAPE"""
    global PLANNER_ID, RUL_SHAPE_P
    PLANNER_ID = planner_id
    rospy.loginfo("==== Batch start: {} ====".format(PLANNER_ID))

    for shape in SHAPE_LIST:
        for idx in range(1, RUNS_PER_SHAPE + 1):
            rospy.loginfo("---- Run {}: shape={} ({}) ----".format(idx, shape, prefix))
            # 每次实验前：重置全局、刷新文件名、清空旧 rul.csv
            _reset_globals_for_run()
            _set_paths_for_new_run()
            if os.path.exists(RUL_CSV_PATH):
                try:
                    os.remove(RUL_CSV_PATH)
                except Exception as e:
                    rospy.logwarn("Cannot remove old rul.csv: {}".format(e))

            # 设置退化曲线参数
            RUL_SHAPE_P = [float(shape)] * NUM_JOINTS

            # 基于“当前真实RUL”反推累计角度与 RUL
            bootstrap_used_motor_rul(STARTING_RUL)
            _update_gamma_from_rul_cv()
            _ensure_rul_csv_initialized()

            # 运行一轮实验（保持原有逻辑）
            _run_one_experiment()

            # 移动/重命名输出
            _move_outputs(prefix, shape, idx, export_dir)

def _run_suite(starting_rul_list, max_tasks_value, export_dir):
    """一个阶段：设置 STARTING_RUL & max_tasks & 出口目录，然后跑两种 planner"""
    global STARTING_RUL, max_tasks
    STARTING_RUL = list(map(float, starting_rul_list))
    max_tasks = int(max_tasks_value)
    rospy.logwarn("=== Suite === STARTING_RUL={}, max_tasks={}, export_dir={}".format(STARTING_RUL, max_tasks, export_dir))

    # RULRRTstar
    move_group.set_planner_id("RULRRTstar")
    _run_batch(planner_id="RULRRTstar", prefix="rul-aware", export_dir=export_dir)

    # BaseRRTstar
    move_group.set_planner_id("BaseRRTstar")
    _run_batch(planner_id="BaseRRTstar",  prefix="base",      export_dir=export_dir)

# ====================== 主程序入口 ======================
if __name__ == "__main__":
    try:
        random.seed(SEED)
        np.random.seed(SEED)

        moveit_commander.roscpp_initialize([])
        rospy.init_node('ur5_multi_pick_place_local_deg_py2', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        move_group = moveit_commander.MoveGroupCommander(PLANNING_GROUP)
        move_group.set_num_planning_attempts(PLANNING_ATTEMPTS)
        move_group.set_planning_time(PLANNING_TIME)

        marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10, latch=True)
        start_exit_listener(move_group)

        # 阶段A：Local_Degradation_System
        _run_suite(
            starting_rul_list=[100.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0],
            max_tasks_value=500,
            export_dir=EXPORT_DIR_STAGE_A
        )

        # 阶段B：Full_Health_System
        _run_suite(
            starting_rul_list=[1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0],
            max_tasks_value=80,
            export_dir=EXPORT_DIR_STAGE_B
        )

        # 阶段C：Heterogeneous_Degradation_System
        _run_suite(
            starting_rul_list=[500,800,400,600,1000,700],
            max_tasks_value=80,
            export_dir=EXPORT_DIR_STAGE_C
        )

    finally:
        rospy.loginfo("Shutdown complete.")
