# -*- coding: utf-8 -*-
import numpy as np
np.set_printoptions(suppress=True, formatter={'float': '{:.4f}'.format})

def compute_gamma_from_rul_enhanced(
    RUL,
    *,
    # --- 数值与曲线 ---
    eps=1e-9,
    tiny=1e-6,
    p_max=1.0, c=1e-4, k=3,      # S 型 p(B) 的参数
    # --- 增强控制 ---
    boost=True,                  # 开增强：目标对比度 + 锐化 + 上下限
    r_target=2.0,                # 目标 γ 对比度（minRUL: maxRUL ≈ r_target : 1）
    p_cap=1e4,                   # p 上限
    tau=1.8,                     # 锐化幂（>1 稳定区 1.5~2.0）
    gmin=0.10, gmax=2.50,        # γ 上下限
    # --- 平滑/展示 ---
    rho=0.0, prev_gamma=None,    # 可选 EMA
    alpha=1.0, lam=1.0,          # 用于计算 α+λγ（便于直观看到代价系数）
    debug=False
):
    """
    输入:
      RUL: list/ndarray, 各关节 RUL（正数）
    返回:
      B, p, gamma, cost_coeff, extras(dict)
        - gamma: 均值=1 的最终 γ
        - cost_coeff: α + λγ（逐关节）
        - extras: 额外中间量（R_norm, ratio 等）
    """
    R = np.asarray(RUL, dtype=float)
    J = len(R)

    # --- 异常处理 ---
    if R.size == 0 or np.any(R <= 0) or np.any(~np.isfinite(R)):
        gamma = np.ones(J, dtype=float)
        return 0.0, 1.0, gamma, alpha + lam * gamma, {"note": "invalid RUL -> gamma=1"}

    # --- 归一化到 [0,1]（避免极小值） ---
    R_norm = R / (np.max(R) + eps)
    R_norm = np.clip(R_norm, 1e-3, 1.0)

    # --- 计算 B 与 p(B)（S 型） ---
    mean_R = float(np.mean(R_norm))
    std_R  = float(np.std(R_norm))
    B = std_R / (mean_R + eps)
    p = 1.0 + p_max * (B ** k) / (B ** k + c ** k + 1e-9)

    # --- 增强：对比度保障（反推 p） ---
    if boost:
        ratio = float(np.max(R) / (np.min(R) + eps))  # 用原始 R
        if ratio > 1.0 + 1e-12 and r_target is not None:
            p_gap = float(np.log(r_target) / np.log(ratio))
            p = float(np.clip(max(p, p_gap), 1.0, p_cap))
    else:
        ratio = float(np.max(R) / (np.min(R) + eps))

    # --- 基础权重 + 均值归一（mean=1） ---
    w = (1.0 / np.maximum(R_norm, tiny)) ** p
    w = np.clip(w, 1e-12, 1e12)
    s = float(np.sum(w))
    gamma_target = (J * w / (s + eps)) if s >= 1e-12 else np.ones(J, dtype=float)

    # --- 锐化 + 上下限 + 再归一 ---
    if boost:
        if tau is not None and tau > 1.0:
            g2 = gamma_target ** tau
            gamma_target = J * g2 / (float(np.sum(g2)) + eps)

        if gmin is not None and gmax is not None and gmax > gmin:
            gamma_target = np.clip(gamma_target, gmin, gmax)
            gamma_target = J * gamma_target / (float(np.sum(gamma_target)) + eps)

    # --- EMA 平滑（可选） ---
    if rho > 0.0 and prev_gamma is not None and len(prev_gamma) == J:
        old = np.asarray(prev_gamma, dtype=float)
        gamma = (1 - rho) * old + rho * gamma_target
        gamma = J * gamma / (float(np.sum(gamma)) + eps)
    else:
        gamma = gamma_target

    cost_coeff = alpha + lam * gamma

    if debug:
        print(f"[DEBUG] B={B:.6f}, p={p:.6f}, ratio={ratio:.6f}")
        print(f"[DEBUG] R_norm={R_norm}")
        print(f"[DEBUG] gamma(raw)={gamma_target}")

    extras = {
        "R_norm": R_norm,
        "ratio": ratio,
        "B": B,
        "p": p
    }
    return B, p, gamma, cost_coeff, extras


# ================= 使用示例 =================
if __name__ == "__main__":
    # 你可以随便换 RUL 试
    # RUL = [700,400,1000,600,300,800]
    # RUL = [400,800,300,500,1000,600]
    # RUL = [100,1000,1000,1000,1000,1000]
    RUL = [838.61, 841.65, 829.06, 839.77, 833.27, 837.03]

    B, p, gamma, cost, extras = compute_gamma_from_rul_enhanced(
        RUL,
        p_max=1.0, c=0.0001, k=3,
        boost=True, r_target=2.0, p_cap=10000.0,
        tau=1.8, gmin=0.10, gmax=2.50,
        rho=0.0, prev_gamma=None,
        alpha=1.0, lam=1.0,
        debug=True
    )

    print(f"B = {B:.4f}")
    print(f"p(B,boosted) = {p:.4f}")
    print("RUL:", np.array(RUL, dtype=float))
    print("γ 系数:", np.round(gamma, 4))
    # 如需看到 α+λγ：
    # print("α+λγ 系数:", np.round(cost, 4))
