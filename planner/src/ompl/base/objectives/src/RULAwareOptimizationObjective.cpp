// #include "ompl/base/objectives/RULAwareOptimizationObjective.h"
#include "ompl/base/objectives/RULAwareOptimizationObjective.h"

#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include <queue>
#include <cmath>
#include <numeric>

#include <fstream>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <algorithm>
#include <type_traits>
#include <utility>

#include <cstring>


extern bool     g_rul_dbg_print;
extern uint64_t g_rul_dbg_seq;

extern int      g_rul_dbg_iter;
extern int      g_rul_dbg_cand_idx;   // 未排序前的原始下标
extern int      g_rul_dbg_cand_rank;  // 排序后的名次
extern int      g_rul_dbg_from_id;
extern int      g_rul_dbg_to_id;
extern int      g_rul_dbg_newnode_seq; // 可选：第几个新节点
extern const char* g_rul_dbg_site;     // "SelectParent"/"SortedParent"/"ChosenParent"


namespace ompl
{
namespace base
{

static inline double shortestAngularDelta(double a, double b)
{
    return std::remainder(b - a, 2.0 * M_PI); // (-π,π]
}

// 尺寸对齐：把 RUL/gamma/mask 的长度调到维度大小（不足则重复/填充，超出则截断）
void RULAwareOptimizationObjective::clampSizesToDim()
{
    const std::size_t J = si_->getStateDimension();
    auto fix = [J](auto &v, typename std::remove_reference<decltype(v)>::type::value_type fill){
        if (v.empty()) { v.assign(J, fill); return; }
        if (v.size() < J) {
            // 重复最后一个填满
            v.resize(J, v.back());
        } else if (v.size() > J) {
            v.resize(J);
        }
    };
    fix(RUL_,   1000.0);
    fix(gamma_, 1.0);
    // mask 默认 false（安全：按线性差）
    if (isRevoluteContinuous_.empty()) isRevoluteContinuous_.assign(J, false);
    else if (isRevoluteContinuous_.size() != J) isRevoluteContinuous_.resize(J, false);
}

// 自动推断：SO2 子空间 → true；其他 → false
void RULAwareOptimizationObjective::inferRevoluteMaskFromStateSpace()
{
    const std::size_t J = si_->getStateDimension();
    std::vector<bool> mask(J, false); // 默认 false

    // 复合空间：逐子空间判断
    if (auto c = std::dynamic_pointer_cast<CompoundStateSpace>(si_->getStateSpace())) {
        unsigned idx = 0;
        for (unsigned i = 0; i < c->getSubspaceCount(); ++i) {
            auto sub = c->getSubspace(i);
            const unsigned d = sub->getDimension();
            if (std::dynamic_pointer_cast<SO2StateSpace>(sub)) {
                for (unsigned k = 0; k < d; ++k) mask[idx + k] = true; // SO2 → wrap
            } else {
                for (unsigned k = 0; k < d; ++k) mask[idx + k] = false; // 非SO2 → 线性
            }
            idx += d;
        }
    } else {
        // 非复合：若就是 SO2 → true；否则 false
        mask.assign(J, (bool)std::dynamic_pointer_cast<SO2StateSpace>(si_->getStateSpace()));
    }

    isRevoluteContinuous_ = std::move(mask);
    clampSizesToDim();
}

// 原构造
RULAwareOptimizationObjective::RULAwareOptimizationObjective(
    const SpaceInformationPtr &si,
    const std::vector<double> &RUL_input,
    const std::vector<double> &gamma_input,
    double lambda,
    double alpha,
    double rul_min,
    double epsilon)
  : OptimizationObjective(si)
  , RUL_(RUL_input)
  , gamma_(gamma_input)
  , lambda_(lambda)
  , alpha_(alpha)
  , RULmin_(rul_min)
  , epsilon_(epsilon)
{
    description_ = "RUL-aware path length";
    setCostToGoHeuristic(goalRegionCostToGo);

    // 默认先全 false（线性差），然后尝试自动推断
    isRevoluteContinuous_.clear();
    clampSizesToDim();
    inferRevoluteMaskFromStateSpace(); // 可以被后续 setRevoluteMask 覆盖
}

// ✅ 新增：带 mask 的构造
RULAwareOptimizationObjective::RULAwareOptimizationObjective(
    const SpaceInformationPtr &si,
    const std::vector<double> &RUL_input,
    const std::vector<double> &gamma_input,
    const std::vector<bool>  &revoluteMask,
    double lambda,
    double alpha,
    double rul_min,
    double epsilon)
  : OptimizationObjective(si)
  , RUL_(RUL_input)
  , gamma_(gamma_input)
  , lambda_(lambda)
  , alpha_(alpha)
  , RULmin_(rul_min)
  , epsilon_(epsilon)
  , isRevoluteContinuous_(revoluteMask)
{
    description_ = "RUL-aware path length";
    setCostToGoHeuristic(goalRegionCostToGo);
    clampSizesToDim(); // 把传入的向量对齐到维度大小
}

Cost RULAwareOptimizationObjective::stateCost(const State */*s*/) const
{
    return identityCost();
}

// Cost RULAwareOptimizationObjective::motionCost(const State *s1, const State *s2) const
// {
//     const auto *q1 = s1->as<RealVectorStateSpace::StateType>();
//     const auto *q2 = s2->as<RealVectorStateSpace::StateType>();

//     double dist = 0.0;
//     double rulPenalty = 0.0;

//     const std::size_t J = RUL_.size();

//     // ====== 1) 动态获取当前批次的 RUL 最大最小值 ======
//     double RUL_min_cur = *std::min_element(RUL_.begin(), RUL_.end());
//     double RUL_max_cur = *std::max_element(RUL_.begin(), RUL_.end());
//     if (RUL_max_cur == RUL_min_cur)
//         RUL_max_cur = RUL_min_cur + 1e-6; // 防止除零

//     // ====== 2) 遍历每个关节 ======
//     for (std::size_t i = 0; i < J; ++i)
//     {
//         const bool useSO2 = (i < isRevoluteContinuous_.size()) ? isRevoluteContinuous_[i] : false;
//         const double dq   = useSO2 ? shortestAngularDelta(q1->values[i], q2->values[i])
//                                    : (q2->values[i] - q1->values[i]);

//         // --- 3) RUL 归一化到 [0,1] ---
//         double rul_norm = (RUL_[i] - RUL_min_cur) / (RUL_max_cur - RUL_min_cur);
//         rul_norm = std::max(0.0, std::min(1.0, rul_norm));  // 限制在 [0,1] 内

//         // --- 4) 代价累加：c = Σ |Δq_i| * (α + λ * γ_i) / (rul_norm + ε) ---
//         const double w = std::fabs(dq) / (rul_norm + epsilon_);
//         dist       += alpha_    * w;
//         rulPenalty += gamma_[i] * w;
//     }

//     // ====== 5) 返回总代价 ======
//     return Cost(dist + lambda_ * rulPenalty);
// }

Cost RULAwareOptimizationObjective::motionCost(const State *s1, const State *s2) const
{
    const auto *q1 = s1->as<RealVectorStateSpace::StateType>();
    const auto *q2 = s2->as<RealVectorStateSpace::StateType>();

    double dist = 0.0;
    double rulPenalty = 0.0;

    const std::size_t J = RUL_.size();
    const double RUL_floor = 1e-6;

    // ====== 在函数内部定义模式 ======
    // 可选："none" / "minmax" / "fixed1000"
    std::string mode = "fixed1000";   // 👉 这里直接改，就能切换模式

    // ====== minmax 情况要先算全局 min/max ======
    double RUL_min_cur = *std::min_element(RUL_.begin(), RUL_.end());
    double RUL_max_cur = *std::max_element(RUL_.begin(), RUL_.end());
    if (RUL_max_cur == RUL_min_cur)
        RUL_max_cur = RUL_min_cur + 1e-6;

    // --- 新增：逐关节缓存（用于循环后“一行写出”） ---
    std::vector<double> dq_v(J), rul_norm_v(J), w_v(J), dist_v(J), pen_v(J);

    for (std::size_t i = 0; i < J; ++i)
    {
        const bool useSO2 = (i < isRevoluteContinuous_.size()) ? isRevoluteContinuous_[i] : false;
        const double dq   = useSO2 ? shortestAngularDelta(q1->values[i], q2->values[i])
                                   : (q2->values[i] - q1->values[i]);

        double rul_norm = 1.0;

        // ====== 三种模式 ======
        if (mode == "none") {
            // 不归一化
            rul_norm = std::max(RUL_[i], RUL_floor);

        } else if (mode == "minmax") {
            // 最大最小归一化
            rul_norm = (RUL_[i] - RUL_min_cur) / (RUL_max_cur - RUL_min_cur);
            rul_norm = std::max(rul_norm, 0.05);

        } else if (mode == "fixed1000") {
            // 固定归一化
            rul_norm = RUL_[i] / 1000.0;
            rul_norm = std::max(rul_norm, 0.05);
        }

        // plan 1 ====== 基础代价 ======
        double w = std::fabs(dq) / (rul_norm + epsilon_);
        double dist_i = alpha_ * w;
        double penalty_i = gamma_[i] * w;

        dist       += dist_i;
        rulPenalty += penalty_i;

        // --- 缓存本关节量 ---
        dq_v[i]        = dq;
        rul_norm_v[i]  = rul_norm;
        w_v[i]         = w;
        dist_v[i]      = dist_i;
        pen_v[i]       = penalty_i;

        // ====== 调试打印（保留注释） ======
        // OMPL_INFORM("joint %zu | dq=%.6f | RUL=%.3f | rul_norm=%.6f | w=%.6e | dist_i=%.6e | penalty_i=%.6e | gamma=%.3f",
        //             i, dq, RUL_[i], rul_norm, w, dist_i, penalty_i, gamma_[i]);
    }

    // ====== 循环后：一次性写“一行/一次调用”的 CSV ======
    if (g_rul_dbg_print && g_rul_dbg_site &&
        (!std::strcmp(g_rul_dbg_site,"SelectParent") ||
         !std::strcmp(g_rul_dbg_site,"SortedParent")  ||
         !std::strcmp(g_rul_dbg_site,"ChosenParent")))
    {
        const char* debug_path = std::getenv("RUL_DEBUG_CSV_PATH");
        static std::ofstream csv(debug_path && *debug_path ? debug_path : "/dev/null", std::ios::app);
        static bool header = false;
        if (!header && csv.good()) {
            // 基础元信息列
            csv << "seq,iter,site,new_node_seq,to_id,from_id,cand_idx,cand_rank";
            // 逐关节列（六个电机一行）：dq/RUL/rul_norm/gamma/w/dist_i/penalty_i
            for (std::size_t j = 0; j < J; ++j) csv << ",dq"        << j;
            for (std::size_t j = 0; j < J; ++j) csv << ",RUL"       << j;
            for (std::size_t j = 0; j < J; ++j) csv << ",rul_norm"  << j;
            for (std::size_t j = 0; j < J; ++j) csv << ",gamma"     << j;
            for (std::size_t j = 0; j < J; ++j) csv << ",w"         << j;
            for (std::size_t j = 0; j < J; ++j) csv << ",dist_i"    << j;
            for (std::size_t j = 0; j < J; ++j) csv << ",penalty_i" << j;
            // 新增两列：总和
            csv << ",sum_dist,sum_penalty\n";
            header = true;
        }

        // 写一行
        csv << g_rul_dbg_seq          << ","
            << g_rul_dbg_iter         << ","
            << g_rul_dbg_site         << ","
            << g_rul_dbg_newnode_seq  << ","
            << g_rul_dbg_to_id        << ","
            << g_rul_dbg_from_id      << ","
            << g_rul_dbg_cand_idx     << ","
            << g_rul_dbg_cand_rank;

        csv << std::fixed;
        // dq0..dq5
        for (std::size_t j = 0; j < J; ++j) csv << "," << std::setprecision(6) << dq_v[j];
        // RUL0..RUL5
        for (std::size_t j = 0; j < J; ++j) csv << "," << std::setprecision(3) << RUL_[j];
        // rul_norm0..5
        for (std::size_t j = 0; j < J; ++j) csv << "," << std::setprecision(6) << rul_norm_v[j];
        // gamma0..5
        for (std::size_t j = 0; j < J; ++j) csv << "," << std::setprecision(3) << gamma_[j];
        // w0..5
        for (std::size_t j = 0; j < J; ++j) csv << "," << std::setprecision(6) << w_v[j];
        // dist_i0..5
        for (std::size_t j = 0; j < J; ++j) csv << "," << std::setprecision(6) << dist_v[j];
        // penalty_i0..5
        for (std::size_t j = 0; j < J; ++j) csv << "," << std::setprecision(6) << pen_v[j];

        // 新增两列：求和
        csv << "," << std::setprecision(6) << dist      // = sum(dist_v)
            << "," << std::setprecision(6) << rulPenalty; // = sum(pen_v)

        csv << "\n";
        csv.flush();
    }

    double totalCost = dist + lambda_ * rulPenalty;

    // ====== 总结打印（保留注释） ======
    // OMPL_INFORM("Summary | dist=%.6f | rulPenalty=%.6f | lambda=%.3f | totalCost=%.6f",
    //             dist, rulPenalty, lambda_, totalCost);

    return Cost(totalCost);
}





// 低估的启发式即可（用空间距离）
Cost RULAwareOptimizationObjective::motionCostHeuristic(const State *s1, const State *s2) const
{
    return Cost(si_->distance(s1, s2));
}

} // namespace base
} // namespace ompl
