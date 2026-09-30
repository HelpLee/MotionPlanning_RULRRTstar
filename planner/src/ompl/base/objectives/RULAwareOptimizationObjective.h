#ifndef OMPL_BASE_OBJECTIVES_RUL_AWARE_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_RUL_AWARE_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/Cost.h"


#include <vector>
#include <algorithm>

namespace ompl
{
namespace base
{
class RULAwareOptimizationObjective : public OptimizationObjective
{
public:
    // 原构造
    RULAwareOptimizationObjective(const SpaceInformationPtr &si,
                                  const std::vector<double> &RUL,
                                  const std::vector<double> &gamma,
                                  double lambda,
                                  double alpha   = 0.2,
                                  double rul_min = 10.0,
                                  double epsilon = 1e-9);

    // ✅ 新增：直接传 mask 的构造（true=按SO2最短弧，false=线性差）
    RULAwareOptimizationObjective(const SpaceInformationPtr &si,
                                  const std::vector<double> &RUL,
                                  const std::vector<double> &gamma,
                                  const std::vector<bool>  &revoluteMask,
                                  double lambda,
                                  double alpha   = 0.2,
                                  double rul_min = 10.0,
                                  double epsilon = 1e-9);

    Cost stateCost(const State *s) const override;
    Cost motionCost(const State *s1, const State *s2) const override;
    Cost motionCostHeuristic(const State *s1, const State *s2) const override;

    // -------- Runtime setters / getters --------
    void setRUL(const std::vector<double> &r)   { RUL_ = r;   clampSizesToDim(); }
    void setGamma(const std::vector<double> &g) { gamma_ = g; clampSizesToDim(); }
    void setLambda(double l)                    { lambda_ = l; }
    void setAlpha(double a)                     { alpha_ = a; }
    void setRULmin(double rmin)                 { RULmin_ = rmin; }
    void setEpsilon(double e)                   { epsilon_ = e; }

    const std::vector<double>& getRUL()   const { return RUL_; }
    const std::vector<double>& getGamma() const { return gamma_; }
    double getLambda()  const { return lambda_; }
    double getAlpha()   const { return alpha_; }
    double getRULmin()  const { return RULmin_; }
    double getEpsilon() const { return epsilon_; }

    /** 指定哪些维度按连续旋转（SO2）处理；true=SO2最短弧，false=线性差 */
    void setRevoluteMask(const std::vector<bool> &mask) { isRevoluteContinuous_ = mask; clampSizesToDim(); }
    const std::vector<bool>& getRevoluteMask() const    { return isRevoluteContinuous_; }

    /** ✅ 新增：从 StateSpace 自动推断（SO2→true，其他→false） */
    void inferRevoluteMaskFromStateSpace();

private:
    void clampSizesToDim(); // 把 RUL/gamma/mask 尺寸调到 si_->getStateDimension()

private:
    std::vector<double> RUL_;     // RUL_j
    std::vector<double> gamma_;   // γ_j
    double lambda_;               // λ
    double alpha_;                // α
    double RULmin_;               // R_min
    double epsilon_;              // ε

    // true: 按 SO2（可绕圈）最短角差；false：线性差（有限位/直线）
    std::vector<bool> isRevoluteContinuous_;
};
} // namespace base
} // namespace ompl

#endif
