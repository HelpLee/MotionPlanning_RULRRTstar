/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres, Ioan Sucan, Javier V Gomez, Jonathan Gammell */

#include "ompl/geometric/planners/rrt/RULRRTstar.h"
// #include "RULRRTstar/RULRRTstar.h"

#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <vector>
#include "ompl/base/Goal.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
// #include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/RULAwareOptimizationObjective.h"
// #include "RULRRTstar/RULAwareOptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/base/samplers/informed/OrderedInfSampler.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/GeometricEquations.h"
// #include <ros/param.h>
// #include <XmlRpcValue.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <ompl/util/RandomNumbers.h>
#include <ros/ros.h>

#include <cmath>       // std::remainder, std::fmod
#include <numeric>     // std::accumulate
#include <sstream>     // std::ostringstream
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include <fstream>
#include <cstdlib>
#include <cstdint>
#include <map>      // ✅ 你用到了 std::map
#include <string>

#include <thread>
#include <chrono>
#include <iomanip>  // ✅ 加到文件开头

bool     g_rul_dbg_print = false;
uint64_t g_rul_dbg_seq   = 0;

int      g_rul_dbg_iter       = -1;
int      g_rul_dbg_cand_idx   = -1;
int      g_rul_dbg_cand_rank  = -1;
int      g_rul_dbg_from_id    = -1;
int      g_rul_dbg_to_id      = -1;
int      g_rul_dbg_newnode_seq= 0;
const char* g_rul_dbg_site    = nullptr;





// ======== 简单硬编码：CSV 路径 ========
// Set RUL_CSV_PATH before starting move_group; keep the historical default for old workspaces.
static const char* kRulCsvPath = []() {
    const char* path = std::getenv("RUL_CSV_PATH");
    return path && *path ? path : "/home/haibo/catkin_ws/src/pick_place_demo/src/joint_usage/rul.csv";
}();

// 解析一行到 double 向量，支持逗号或空白分隔
static void parseLineToDoubles(const std::string& line, std::vector<double>& out)
{
    out.clear();
    if (line.empty()) return;

    auto trim = [](std::string s) {
        while (!s.empty() && (s.back()=='\r' || s.back()==' ' || s.back()=='\t')) s.pop_back();
        size_t i=0; while (i<s.size() && (s[i]==' ' || s[i]=='\t')) ++i;
        return s.substr(i);
    };

    std::string L = trim(line);

    auto splitAndParse = [&](char sep) {
        std::stringstream ss(L);
        std::string tok;
        if (sep == '\0') {
            // 空白分隔
            while (ss >> tok) {
                std::istringstream is(tok);
                double v; if (is >> v) out.push_back(v);
            }
        } else {
            // 指定分隔符
            while (std::getline(ss, tok, sep)) {
                tok = trim(tok);
                if (tok.empty()) continue;
                std::istringstream is(tok);
                double v; if (is >> v) out.push_back(v);
            }
        }
    };

    if (L.find(',') != std::string::npos) splitAndParse(',');
    else                                   splitAndParse('\0');
}

// 只读 CSV 的“最后一行”（忽略空行）；带轻量重试，避免并发写入造成半行
static bool readLastLineRUL(const std::string& path, std::vector<double>& out, int retries = 3)
{
    using namespace std::chrono_literals;
    for (int attempt = 0; attempt < retries; ++attempt)
    {
        out.clear();
        std::ifstream ifs(path);
        if (!ifs) return false;

        std::string line, lastNonEmpty;
        while (std::getline(ifs, line)) {
            while (!line.empty() && (line.back()=='\r' || line.back()==' ' || line.back()=='\t'))
                line.pop_back();
            if (!line.empty()) lastNonEmpty = line;
        }

        if (lastNonEmpty.empty()) return false;

        std::vector<double> vals;
        parseLineToDoubles(lastNonEmpty, vals);
        if (!vals.empty()) { out.swap(vals); return true; }

        // 解析失败就稍等再试（可能遇到写入竞争）
        std::this_thread::sleep_for(10ms);
    }
    return false;
}


ompl::geometric::RULRRTstar::RULRRTstar(const base::SpaceInformationPtr &si)
  : base::Planner(si, "RULRRTstar")
{

    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.canReportIntermediateSolutions = true;

    Planner::declareParam<double>("range", this, &RULRRTstar::setRange, &RULRRTstar::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RULRRTstar::setGoalBias, &RULRRTstar::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("rewire_factor", this, &RULRRTstar::setRewireFactor, &RULRRTstar::getRewireFactor,
                                  "1.0:0.01:2.0");
    Planner::declareParam<bool>("use_k_nearest", this, &RULRRTstar::setKNearest, &RULRRTstar::getKNearest, "0,1");
    Planner::declareParam<bool>("delay_collision_checking", this, &RULRRTstar::setDelayCC, &RULRRTstar::getDelayCC, "0,1");
    Planner::declareParam<bool>("tree_pruning", this, &RULRRTstar::setTreePruning, &RULRRTstar::getTreePruning, "0,1");
    Planner::declareParam<double>("prune_threshold", this, &RULRRTstar::setPruneThreshold, &RULRRTstar::getPruneThreshold,
                                  "0.:.01:1.");
    Planner::declareParam<bool>("pruned_measure", this, &RULRRTstar::setPrunedMeasure, &RULRRTstar::getPrunedMeasure, "0,1");
    Planner::declareParam<bool>("informed_sampling", this, &RULRRTstar::setInformedSampling, &RULRRTstar::getInformedSampling,
                                "0,1");
    Planner::declareParam<bool>("sample_rejection", this, &RULRRTstar::setSampleRejection, &RULRRTstar::getSampleRejection,
                                "0,1");
    Planner::declareParam<bool>("new_state_rejection", this, &RULRRTstar::setNewStateRejection,
                                &RULRRTstar::getNewStateRejection, "0,1");
    Planner::declareParam<bool>("use_admissible_heuristic", this, &RULRRTstar::setAdmissibleCostToCome,
                                &RULRRTstar::getAdmissibleCostToCome, "0,1");
    Planner::declareParam<bool>("ordered_sampling", this, &RULRRTstar::setOrderedSampling, &RULRRTstar::getOrderedSampling,
                                "0,1");
    Planner::declareParam<unsigned int>("ordering_batch_size", this, &RULRRTstar::setBatchSize, &RULRRTstar::getBatchSize,
                                        "1:100:1000000");
    Planner::declareParam<bool>("focus_search", this, &RULRRTstar::setFocusSearch, &RULRRTstar::getFocusSearch, "0,1");
    Planner::declareParam<unsigned int>("number_sampling_attempts", this, &RULRRTstar::setNumSamplingAttempts,
                                        &RULRRTstar::getNumSamplingAttempts, "10:10:100000");


    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
    addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });

}   

ompl::geometric::RULRRTstar::~RULRRTstar()
{
    freeMemory();
}

void ompl::geometric::RULRRTstar::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName()); // 
    sc.configurePlannerRange(maxDistance_);
    if (!si_->getStateSpace()->hasSymmetricDistance() || !si_->getStateSpace()->hasSymmetricInterpolate())
    {
        OMPL_WARN("%s requires a state space with symmetric distance and symmetric interpolation.", getName().c_str());
    }

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    // nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    nn_->setDistanceFunction([this](const Motion* A, const Motion* B) {return rulAwareDistance(A, B);});


    if (pdef_)
    {
        // 维度自适应
        const std::size_t J = si_->getStateDimension();

        // 1) 默认参数
        std::vector<double> rul_values(J, 1000.0);
        std::vector<double> gamma_values(J, 1.0);
        double lambda = 10.0, alpha = 1.0, rulmin = 1.0, eps = 1e-9;

        // 2) 尝试读取 CSV
        std::vector<double> csv_vals;
        if (readLastLineRUL(kRulCsvPath, csv_vals)) {
            if (csv_vals.size() >= 2*J + 4) {
                // RUL
                rul_values.assign(csv_vals.begin(), csv_vals.begin() + J);
                // gamma
                gamma_values.assign(csv_vals.begin() + J, csv_vals.begin() + 2*J);
                // 其他参数
                lambda = csv_vals[2*J + 0];
                alpha  = csv_vals[2*J + 1];
                rulmin = csv_vals[2*J + 2];
                eps    = csv_vals[2*J + 3];
                OMPL_INFORM("[RULRRTstar] Loaded all parameters from CSV (%s).", kRulCsvPath);
            } else {
                OMPL_WARN("[RULRRTstar] CSV has %zu values (< %zu). Using defaults.",
                        csv_vals.size(), 2*J+4);
            }
        } else {
            OMPL_WARN("[RULRRTstar] Cannot read CSV at setup: %s. Using defaults.", kRulCsvPath);
        }

        // 3) 创建并挂载目标
        auto obj = std::make_shared<ompl::base::RULAwareOptimizationObjective>(
            si_, rul_values, gamma_values, lambda, alpha, rulmin, eps
        );

        // UR5: 六个有限角度旋转关节 → 线性差
        std::vector<bool> revoluteMask(J, false);
        obj->setRevoluteMask(revoluteMask);

        pdef_->setOptimizationObjective(obj);
        opt_ = pdef_->getOptimizationObjective();

        // 打印日志
        std::ostringstream oss;
        for (size_t i = 0; i < rul_values.size(); ++i) { if (i) oss << ", "; oss << rul_values[i]; }
        OMPL_INFORM("[RULRRTstar] RUL:   [%s]", oss.str().c_str());
        OMPL_INFORM("[RULRRTstar] lambda=%.3f, alpha=%.3f, RULmin=%.3f, eps=%.3e",
                    lambda, alpha, rulmin, eps);

        bestCost_  = opt_->infiniteCost();
        prunedCost_= opt_->infiniteCost();
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }



    // 🧠 空间测度 & 重接常数
    // Get the measure of the entire space:
    prunedMeasure_ = si_->getSpaceMeasure(); 
    // Calculate some constants:
    calculateRewiringLowerBounds();  

}

void ompl::geometric::RULRRTstar::clear() 
{
    setup_ = false;
    Planner::clear();
    sampler_.reset();
    infSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();

    bestGoalMotion_ = nullptr;
    goalMotions_.clear();
    startMotions_.clear();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedMeasure_ = 0.0;
}


ompl::base::PlannerStatus ompl::geometric::RULRRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
    static int nodeCounter = 0;
    static std::map<Motion*, int> nodeIDs;

    // 在每次 solve() 前尝试热更新 RUL
    if (auto rulObj = std::dynamic_pointer_cast<ompl::base::RULAwareOptimizationObjective>(opt_))
    {
        const std::size_t J = si_->getStateDimension();   // solve() 是另一个作用域，这里可以再定义一次 J

        std::vector<double> rul_csv_vals;
        if (readLastLineRUL(kRulCsvPath, rul_csv_vals)) {
            if (rul_csv_vals.size() >= J) {
                rul_csv_vals.resize(J);
                rulObj->setRUL(rul_csv_vals);
                OMPL_INFORM("[RULRRTstar] RUL (last line) updated from CSV before solve().");
            } else {
                OMPL_WARN("[RULRRTstar] CSV has %zu values (< %zu). Skip update.", rul_csv_vals.size(), J);
            }
        } else {
            OMPL_WARN("[RULRRTstar] Cannot read RUL CSV in solve(): %s. Keep previous RUL.", kRulCsvPath);
        }

    }


    checkValidity();

    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    bool symCost = opt_->isSymmetric();

    // Check if there are more starts
    if (pis_.haveMoreStartStates() == true)
    {
        // There are, add them
        while (const base::State *st = pis_.nextStart())
        {
            auto *motion = new Motion(si_);
            // 添加编号
            nodeIDs[motion] = nodeCounter++;
            si_->copyState(motion->state, st);
            motion->cost = opt_->identityCost();
            nn_->add(motion);
            startMotions_.push_back(motion);
        }

        // And assure that, if we're using an informed sampler, it's reset
        infSampler_.reset();
    }
    // No else

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Allocate a sampler if necessary
    if (!sampler_ && !infSampler_)
    {
        allocSampler();
    }

    OMPL_INFORM("%s: Started planning with %u states. Seeking a solution better than %.5f.", getName().c_str(), nn_->size(), opt_->getCostThreshold().value());

    if ((useTreePruning_ || useRejectionSampling_ || useInformedSampling_ || useNewStateRejection_) &&
        !si_->getStateSpace()->isMetricSpace())
        OMPL_WARN("%s: The state space (%s) is not metric and as a result the optimization objective may not satisfy "
                  "the triangle inequality. "
                  "You may need to disable pruning or rejection.",
                  getName().c_str(), si_->getStateSpace()->getName().c_str());

    const base::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();

    Motion *approxGoalMotion = nullptr;
    double approxDist = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    std::vector<Motion *> nbh;

    std::vector<base::Cost> costs;
    std::vector<base::Cost> incCosts;
    std::vector<std::size_t> sortedCostIndices;

    std::vector<int> valid;
    unsigned int rewireTest = 0;
    unsigned int statesGenerated = 0;

    if (bestGoalMotion_)
        OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f", getName().c_str(),
                    bestCost_);

    if (useKNearest_)
        OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(),
                    (unsigned int)std::ceil(k_rrt_ * log((double)(nn_->size() + 1u))));
    else
        OMPL_INFORM(
            "%s: Initial rewiring radius of %.2f", getName().c_str(),
            std::min(maxDistance_, r_rrt_ * std::pow(log((double)(nn_->size() + 1u)) / ((double)(nn_->size() + 1u)),
                                                     1 / (double)(si_->getStateDimension()))));

    // our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);

    while (ptc == false)
    {
        iterations_++;

        // sample random state (with goal biasing)
        // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal
        // states.
        if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ &&
            goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
        {
            // Attempt to generate a sample, if we fail (e.g., too many rejection attempts), skip the remainder of this
            // loop and return to try again
            if (!sampleUniform(rstate))
                continue;
        }

        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);

        if (intermediateSolutionCallback && si_->equalStates(nmotion->state, rstate))
            continue;

        base::State *dstate = rstate;

        // find state to add to the tree
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        // Check if the motion between the nearest state and the state to add is valid
        if (si_->checkMotion(nmotion->state, dstate))
        {
            // create a motion
            auto *motion = new Motion(si_);
            // 添加编号
            nodeIDs[motion] = nodeCounter++;
            si_->copyState(motion->state, dstate);
            // Set the parent of the new node 'motion' to be the nearest node 'nmotion'
            motion->parent = nmotion;
            // Calculate the incremental cost from the parent node to the new node
            motion->incCost = opt_->motionCost(nmotion->state, motion->state);
            // Calculate the total cost for the new node by combining the parent's total cost
            // with the incremental cost from the parent to the new node
            motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

            // Find nearby neighbors of the new motion
            getNeighbors(motion, nbh);

            // === Debug: 候选数量 ===
            // const std::size_t cand_cnt = nbh.size();
            // OMPL_INFORM("[ParentSelect] iter=%u  candidates=%zu  use_k_nearest=%d  maxDist=%.3f",
            //             iterations_, cand_cnt, useKNearest_, maxDistance_);

            rewireTest += nbh.size();
            ++statesGenerated; // 成功生成并加入树的节点数加一

            // cache for distance computations
            //
            // Our cost caches only increase in size, so they're only
            // resized if they can't fit the current neighborhood
            if (costs.size() < nbh.size())
            {
                costs.resize(nbh.size());
                incCosts.resize(nbh.size());
                sortedCostIndices.resize(nbh.size());
            }

            // cache for motion validity (only useful in a symmetric space)
            //
            // Our validity caches only increase in size, so they're
            // only resized if they can't fit the current neighborhood
            if (valid.size() < nbh.size())
                valid.resize(nbh.size());
            std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

            // Finding the nearest neighbor to connect to
            // By default, neighborhood states are sorted by cost, and collision checking
            // is performed in increasing order of cost
            // ===================================select parent nodes===============================
            // calculate the total cost from the start to the new node!
            // 先对邻居 nbh 全部算代价（incCosts / costs），按总代价从小到大排序，然后按顺序做碰撞检测，遇到第一个可行的就选为父节点。
            if (delayCC_)
            {
                // calculate all costs and distances
                for (std::size_t i = 0; i < nbh.size(); ++i)
                {
                    incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                    costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                }


                // sort the nodes
                //
                // we're using index-value pairs so that we can get at
                // original, unsorted indices
                for (std::size_t i = 0; i < nbh.size(); ++i)
                    sortedCostIndices[i] = i;
                std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + nbh.size(), compareFn);

                // // ====== 按排序名次，把每个候选的逐关节明细写到 CSV ======
                // {
                //     g_rul_dbg_iter = (int)iterations_;
                //     int new_node_id = nodeIDs[motion];

                //     for (std::size_t k = 0; k < nbh.size(); ++k)
                //     {
                //         std::size_t idx = sortedCostIndices[k];   // 原始下标
                //         ++g_rul_dbg_seq;

                //         g_rul_dbg_site       = "SortedParent";
                //         g_rul_dbg_cand_idx   = (int)idx;          // 未排序前的下标
                //         g_rul_dbg_cand_rank  = (int)k;            // 排序后的名次
                //         g_rul_dbg_from_id    = nodeIDs[nbh[idx]]; // 候选父节点ID
                //         g_rul_dbg_to_id      = new_node_id;       // 新节点ID

                //         g_rul_dbg_print = true;
                //         (void)opt_->motionCost(nbh[idx]->state, motion->state); // 只为写CSV，结果不用
                //         g_rul_dbg_print = false;
                //     }
                // }

                // ====== 按排序名次，把每个候选的逐关节明细写到 CSV ======

                // // 打印排序结果（兼容 melodic）
                // OMPL_INFORM("[select parent nodes | iteration=%zu | Sorted Candidates]", iterations_);

                // for (std::size_t k = 0; k < nbh.size(); ++k)
                // {
                //     std::size_t idx = sortedCostIndices[k];

                //     // Cost 转 double：在 melodic 中里直接用 .value()
                //     double incVal = incCosts[idx].value();
                //     double cumVal = costs[idx].value();

                //     OMPL_INFORM(" iter=%zu  rank=%zu  idx=%zu  inc=%.6f  cum=%.6f",
                //                 iterations_, k, idx, incVal, cumVal);
                // }

                // // —— 逐候选父节点：按当前目标函数逻辑计算并打印（含逐电机明细，fixed 小数） ——
                // if (auto rulObj = std::dynamic_pointer_cast<ompl::base::RULAwareOptimizationObjective>(opt_))
                // {
                //     using ompl::base::RealVectorStateSpace;
                //     const std::size_t J  = si_->getStateDimension();
                //     const auto &RUL      = rulObj->getRUL();
                //     const auto &gamma    = rulObj->getGamma();
                //     const auto &mask     = rulObj->getRevoluteMask();
                //     const double alpha   = rulObj->getAlpha();
                //     const double lambda  = rulObj->getLambda();
                //     const double eps     = rulObj->getEpsilon();

                //     auto ang = [](double a, double b){ return std::remainder(b - a, 2.0 * M_PI); };

                //     {
                //         std::ostringstream head;
                //         head.setf(std::ios::fixed);
                //         head << "[ParentSelect] iter=" << iterations_
                //             << " | candidates=" << nbh.size()
                //             << " (alpha=" << std::setprecision(3) << alpha
                //             << ", lambda=" << lambda << ")";
                //         OMPL_INFORM("%s", head.str().c_str());
                //     }

                //     for (std::size_t k = 0; k < nbh.size(); ++k)
                //     {
                //         const std::size_t idx = sortedCostIndices[k];
                //         const auto *s_from = nbh[idx]->state->as<RealVectorStateSpace::StateType>();
                //         const auto *s_to   = motion->state->as<RealVectorStateSpace::StateType>();

                //         double sum_path = 0.0;          // ∑ α·|dq|
                //         double sum_penalty_raw = 0.0;   // ∑ γ·|dq|/(rul_norm+ε)（未乘 λ）
                //         // 逐电机暂存，方便明细打印
                //         std::vector<double> dq_v(J), rul_norm_v(J), path_v(J), pen_raw_v(J), pen_w_v(J);

                //         for (std::size_t j = 0; j < J; ++j)
                //         {
                //             const bool useSO2 = (j < mask.size()) ? mask[j] : false;
                //             const double dq   = useSO2 ? ang((*s_from)[j], (*s_to)[j])
                //                                     : ((*s_to)[j] - (*s_from)[j]);
                //             const double absdq = std::fabs(dq);

                //             // —— 与目标函数一致：fixed1000 归一化，并下限 0.05
                //             double rul_norm = RUL[j] / 1000.0;
                //             if (rul_norm < 0.05) rul_norm = 0.05;

                //             const double path_j   = alpha * absdq;                          // α·|dq|
                //             const double pen_raw  = gamma[j] * (absdq / (rul_norm + eps));   // 未乘 λ
                //             const double pen_w    = lambda * pen_raw;                        // 乘 λ

                //             sum_path        += path_j;
                //             sum_penalty_raw += pen_raw;

                //             dq_v[j]        = dq;
                //             rul_norm_v[j]  = rul_norm;
                //             path_v[j]      = path_j;
                //             pen_raw_v[j]   = pen_raw;
                //             pen_w_v[j]     = pen_w;
                //         }

                //         const double sum_rulw = lambda * sum_penalty_raw;
                //         const double total    = sum_path + sum_rulw;

                //         const double incVal = incCosts[idx].value();   // OMPL 实际增量代价
                //         const double cumVal = costs[idx].value();      // OMPL 累计代价

                //         // —— 汇总一行（候选） ——
                //         {
                //             std::ostringstream line;
                //             line.setf(std::ios::fixed);
                //             line << "    ⇒ rank=" << k
                //                 << " idx=" << idx
                //                 << std::setprecision(6)
                //                 << " | path="  << sum_path
                //                 << " RUL(w)="  << sum_rulw
                //                 << " total="   << total
                //                 << " | inc="   << incVal
                //                 << " cum="     << cumVal;
                //             OMPL_INFORM("%s", line.str().c_str());
                //         }

                //         // —— 逐电机明细（每个关节一行） ——
                //         for (std::size_t j = 0; j < J; ++j)
                //         {
                //             std::ostringstream jl;
                //             jl.setf(std::ios::fixed);
                //             jl << "        joint " << j
                //             << " | dq="        << std::setprecision(6) << dq_v[j]
                //             << " | RUL="       << std::setprecision(3) << RUL[j]
                //             << " | rul_norm="  << std::setprecision(6) << rul_norm_v[j]
                //             << " | path_j="    << std::setprecision(6) << path_v[j]
                //             << " | pen_raw="   << std::setprecision(6) << pen_raw_v[j]
                //             << " | pen_w="     << std::setprecision(6) << pen_w_v[j]
                //             << " | gamma="     << std::setprecision(3) << gamma[j];
                //             OMPL_INFORM("%s", jl.str().c_str());
                //         }
                //     }
                // }



                // collision check until a valid motion is found
                //
                // ASYMMETRIC CASE: it's possible that none of these
                // neighbors are valid. This is fine, because motion
                // already has a connection to the tree through
                // nmotion (with populated cost fields!).
                for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                     i != sortedCostIndices.begin() + nbh.size(); ++i)
                {
                    if (nbh[*i] == nmotion ||
                        ((!useKNearest_ || si_->distance(nbh[*i]->state, motion->state) < maxDistance_) &&
                         si_->checkMotion(nbh[*i]->state, motion->state)))
                    {
                        motion->incCost = incCosts[*i];
                        motion->cost = costs[*i];
                        motion->parent = nbh[*i];
                        valid[*i] = 1;
                        break;
                    }
                    else
                        valid[*i] = -1;
                }
            }
            // 从最近的那个开始，当发现某个邻居“代价更好”时，就立刻做碰撞检测；如果可行就更新为候选父节点，继续遍历可能找到更好的。
            else  // if not delayCC
            {
                motion->incCost = opt_->motionCost(nmotion->state, motion->state);
                motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
                // find which one we connect the new state to
                for (std::size_t i = 0; i < nbh.size(); ++i)
                {
                    if (nbh[i] != nmotion)
                    {
                        incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                        costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                        if (opt_->isCostBetterThan(costs[i], motion->cost))
                        {
                            if ((!useKNearest_ || si_->distance(nbh[i]->state, motion->state) < maxDistance_) &&
                                si_->checkMotion(nbh[i]->state, motion->state))
                            {
                                motion->incCost = incCosts[i];
                                motion->cost = costs[i];
                                motion->parent = nbh[i];
                                valid[i] = 1;
                            }
                            else
                                valid[i] = -1;
                        }
                    }
                    else
                    {
                        incCosts[i] = motion->incCost;
                        costs[i] = motion->cost;
                        valid[i] = 1;
                    }
                }
            }

            if (useNewStateRejection_)
            {
                if (opt_->isCostBetterThan(solutionHeuristic(motion), bestCost_))
                {
                    nn_->add(motion);
                    motion->parent->children.push_back(motion);
                }
                else  // If the new motion does not improve the best cost it is ignored.
                {
                    si_->freeState(motion->state);
                    delete motion;
                    continue;
                }
            }
            else
            {
                // add motion to the tree
                nn_->add(motion);
                motion->parent->children.push_back(motion);
            }

            // =================================rewiring==========================================
            // only calculate the increment from the new node to the existing nodes
            
            bool checkForSolution = false;
            for (std::size_t i = 0; i < nbh.size(); ++i)
            {
                if (nbh[i] != motion->parent)
                {
                    base::Cost nbhIncCost;
                    if (symCost)
                        nbhIncCost = incCosts[i];
                    else
                        nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
                    base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
                    if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
                    {
                        bool motionValid;
                        if (valid[i] == 0)
                        {
                            motionValid =
                                (!useKNearest_ || si_->distance(nbh[i]->state, motion->state) < maxDistance_) &&
                                si_->checkMotion(motion->state, nbh[i]->state);
                        }
                        else
                        {
                            motionValid = (valid[i] == 1);
                        }

                        if (motionValid)
                        {
                            // Remove this node from its parent list
                            removeFromParent(nbh[i]);

                            // Add this node to the new parent
                            nbh[i]->parent = motion;
                            nbh[i]->incCost = nbhIncCost;
                            nbh[i]->cost = nbhNewCost;
                            nbh[i]->parent->children.push_back(nbh[i]);

                            // Update the costs of the node's children
                            updateChildCosts(nbh[i]);

                            checkForSolution = true;
                        }
                    }
                }
            }

            // Add the new motion to the goalMotion_ list, if it satisfies the goal
            double distanceFromGoal;
            if (goal->isSatisfied(motion->state, &distanceFromGoal))
            {
                // OMPL_INFORM("========== Solution Found ==========");
                // OMPL_INFORM("[AllSolutions] Iter=%zu  TotalCost=%.6f",
                //             iterations_, motion->cost.value());

                // // 回溯 parent，拿到整条路径
                // std::vector<Motion*> path;
                // Motion* iter = motion;
                // while (iter != nullptr) {
                //     path.push_back(iter);
                //     iter = iter->parent;
                // }

                // // 打印每个节点的累计代价
                // OMPL_INFORM("  Path has %zu nodes", path.size());
                // for (int i = path.size()-1; i >= 0; --i) {
                //     double nodeCost = path[i]->cost.value();
                //     OMPL_INFORM("    Node %d: cost=%.6f", 
                //                 (int)(path.size()-1 - i), nodeCost);

                //     if (i < (int)path.size()-1) {
                //         double inc = path[i]->incCost.value();
                //         OMPL_INFORM("    Incremental cost=%.6f", inc);
                //     }
                // }

                // // 打印路径节点（注意，这里没有插值，只是 RRT* 树上的节点）
                // OMPL_INFORM("  Raw path has %zu nodes", path.size());
                // for (int i = path.size() - 1; i >= 0; --i) {
                //     double nodeCost = path[i]->cost.value();
                //     int globalID = nodeIDs[path[i]];   // 查找全局ID
                //     OMPL_INFORM("    Node %d (ID=%d): cost=%.6f",
                //                 (int)(path.size() - 1 - i), globalID, nodeCost);

                //     if (i < (int)path.size() - 1) {
                //         double inc = path[i]->incCost.value();
                //         OMPL_INFORM("    Incremental cost=%.6f", inc);
                //     }
                // }
                
                // OMPL_INFORM("====================================");

                motion->inGoal = true;
                goalMotions_.push_back(motion);
                checkForSolution = true;
            }

            // Checking for solution or iterative improvement
            if (checkForSolution)
            {
                bool updatedSolution = false;
                if (!bestGoalMotion_ && !goalMotions_.empty())
                {
                    // We have found our first solution, store it as the best. We only add one
                    // vertex at a time, so there can only be one goal vertex at this moment.
                    bestGoalMotion_ = goalMotions_.front();
                    bestCost_ = bestGoalMotion_->cost;
                    updatedSolution = true;

                    OMPL_INFORM("%s: Found an initial solution with a cost of %.2f in %u iterations (%u "
                                "vertices in the graph)",
                                getName().c_str(), bestCost_, iterations_, nn_->size());
                }
                else
                {
                    // We already have a solution, iterate through the list of goal vertices
                    // and see if there's any improvement.
                    for (auto &goalMotion : goalMotions_)
                    {
                        // Is this goal motion better than the (current) best?
                        if (opt_->isCostBetterThan(goalMotion->cost, bestCost_))
                        {
                            bestGoalMotion_ = goalMotion;
                            bestCost_ = bestGoalMotion_->cost;
                            updatedSolution = true;

                            // Check if it satisfies the optimization objective, if it does, break the for loop
                            if (opt_->isSatisfied(bestCost_))
                            {
                                break;
                            }
                        }
                    }
                }

                if (updatedSolution)
                {
                    if (useTreePruning_)
                    {
                        pruneTree(bestCost_);
                    }

                    if (intermediateSolutionCallback)
                    {
                        std::vector<const base::State *> spath;
                        Motion *intermediate_solution =
                            bestGoalMotion_->parent;  // Do not include goal state to simplify code.

                        // Push back until we find the start, but not the start itself
                        while (intermediate_solution->parent != nullptr)
                        {
                            spath.push_back(intermediate_solution->state);
                            intermediate_solution = intermediate_solution->parent;
                        }

                        intermediateSolutionCallback(this, spath, bestCost_);
                    }
                }
            }

            // Checking for approximate solution (closest state found to the goal)
            if (goalMotions_.size() == 0 && distanceFromGoal < approxDist)
            {
                approxGoalMotion = motion;
                approxDist = distanceFromGoal;
            }
        }

        // terminate if a sufficient solution is found
        if (bestGoalMotion_ && opt_->isSatisfied(bestCost_))
            break;
    }

    // Add our solution (if it exists)
    Motion *newSolution = nullptr;
    if (bestGoalMotion_)
    {
        // We have an exact solution
        newSolution = bestGoalMotion_;
    }
    // else if (approxGoalMotion)
    // {
    //     // We don't have a solution, but we do have an approximate solution
    //     newSolution = approxGoalMotion;
    // }
    // No else, we have nothing

    // Add what we found
    if (newSolution)
    {
        ptc.terminate();
        // construct the solution path
        std::vector<Motion *> mpath;
        Motion *iterMotion = newSolution;
        while (iterMotion != nullptr)
        {
            mpath.push_back(iterMotion);
            iterMotion = iterMotion->parent;
        }

        // OMPL_INFORM("===== Final Path (length=%zu) =====", mpath.size());

        // for (int i = mpath.size()-1; i > 0; --i) {
        //     double localPathCost = mpath[i]->incCost.value();   // 父节点 -> 当前节点的局部路径代价
        //     double localRULLoss  = mpath[i-1]->incCost.value(); // 如果你单独算了 rulLoss，可以在这里替换

        //     OMPL_INFORM("  Edge %d -> %d | localPath=%.6f | localRUL=%.6f",
        //                 i, i-1, localPathCost, localRULLoss);
        // }

        // OMPL_INFORM("===================================");



        // OMPL_INFORM("===== Final Path (length=%zu) =====", mpath.size());

        // if (auto *rulObj = dynamic_cast<ompl::base::RULAwareOptimizationObjective*>(opt_.get()))
        // {
        //     const auto &RUL    = rulObj->getRUL();
        //     const auto &gamma  = rulObj->getGamma();
        //     const auto &mask   = rulObj->getRevoluteMask();
        //     const double alpha = rulObj->getAlpha();
        //     const double lambda= rulObj->getLambda();
        //     const double rmin  = rulObj->getRULmin();
        //     const double eps   = rulObj->getEpsilon();

        //     auto shortestAngularDelta = [](double a, double b) {
        //         return std::remainder(b - a, 2.0 * M_PI);
        //     };

        //     double totalPath = 0.0;
        //     double totalRUL  = 0.0;

        //     for (int i = mpath.size()-1; i > 0; --i)
        //     {
        //         auto *s1 = mpath[i]->state->as<ompl::base::RealVectorStateSpace::StateType>();
        //         auto *s2 = mpath[i-1]->state->as<ompl::base::RealVectorStateSpace::StateType>();

        //         double localPath = 0.0;
        //         double localRUL  = 0.0;

        //         for (std::size_t j = 0; j < si_->getStateDimension(); ++j)
        //         {
        //             const bool useSO2 = (j < mask.size()) ? mask[j] : false;
        //             double dq = useSO2 ? shortestAngularDelta((*s1)[j], (*s2)[j])
        //                             : ((*s2)[j] - (*s1)[j]);
        //             double absdq = std::fabs(dq);

        //             // === RUL 归一化方式 ===
        //             double rul_norm = std::max(RUL[j] / 1000.0, 0.05);  // ← 与 motionCost 保持一致

        //             // === 路径代价 ===
        //             double path_j    = alpha * absdq;
        //             // === RUL penalty ===
        //             double rul_pen_j = gamma[j] * (absdq / (rul_norm + eps));
        //             // === 总局部代价 ===
        //             double total_j   = path_j + lambda * rul_pen_j;

        //             localPath += path_j;
        //             localRUL  += rul_pen_j;

        //             std::ostringstream oss;
        //             oss << std::fixed << std::setprecision(6)
        //                 << "Edge " << i << " -> " << i-1
        //                 << " | joint[" << j << "] dq=" << absdq
        //                 << " | RUL=" << RUL[j]
        //                 << " | rul_norm=" << rul_norm
        //                 << " | gamma=" << gamma[j]
        //                 << " | path=" << path_j
        //                 << " | rulPenalty=" << rul_pen_j
        //                 << " | total=" << total_j;
        //             OMPL_INFORM("%s", oss.str().c_str());
        //         }

        //         totalPath += localPath;
        //         totalRUL  += localRUL;

        //         std::ostringstream ossSummary;
        //         ossSummary << std::fixed << std::setprecision(6)
        //                 << "  => localPath=" << localPath
        //                 << " | localRUL=" << localRUL
        //                 << " | alpha=" << alpha
        //                 << " | lambda=" << lambda;
        //         OMPL_INFORM("%s", ossSummary.str().c_str());
        //     }

        //     std::ostringstream ossTotal;
        //     ossTotal << std::fixed << std::setprecision(6)
        //             << "===== Path Summary ====="
        //             << " | totalPath=" << totalPath
        //             << " | totalRUL=" << totalRUL
        //             << " | lambda=" << lambda
        //             << " | finalCost=" << (totalPath + lambda * totalRUL);
        //     OMPL_INFORM("%s", ossTotal.str().c_str());
        // }

        // OMPL_INFORM("===================================");




        // set the solution path
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);

        // =================calculate the total cost of the final path==============================
        double totalCostValue = 0.0;
        auto &states = path->getStates();
        const std::size_t J = si_->getStateDimension();   // ✅ 定义 J
        for (size_t i = 0; i < states.size() - 1; ++i)
        {
            base::Cost incCost = opt_->motionCost(states[i], states[i + 1]);
            totalCostValue += incCost.value();
        }
        OMPL_INFORM("%s: Total path cost (including RUL): %.6f", getName().c_str(), totalCostValue);

        // ============== Angle-Usage (per-joint & total) ======================
        // 说明：
        // - 将整条离散路径按相邻状态做 |Δθ| 累加（对旋转维度用最短角距 [-π, π]）
        // - 如果某些维度是直线关节，请在“直线关节处理”处把 diff 改为 (q2 - q1) 而不是 wrap 到 [-π,π]


        auto toReals = [&](const base::State *s, std::vector<double> &out){
            out.clear();
            out.resize(J);
            auto *q = s->as<ompl::base::RealVectorStateSpace::StateType>();
            for (size_t j = 0; j < J; ++j) out[j] = (*q)[j];
        };

        std::vector<double> perJointUsage(J, 0.0);
        std::vector<bool> mask(J, false);
        if (auto *rulObj = dynamic_cast<ompl::base::RULAwareOptimizationObjective*>(opt_.get()))
            mask = rulObj->getRevoluteMask();

        auto shortestAngularDelta = [](double a, double b){ return std::remainder(b - a, 2.0 * M_PI); };

        const double extent  = si_->getMaximumExtent();
        const double stepLen = 0.02 * extent; // 每段 2% 空间跨度
        std::vector<double> prev, now; prev.reserve(J); now.reserve(J);

        toReals(states.front(), prev);

        for (size_t i = 0; i + 1 < states.size(); ++i) {
            const double L = si_->distance(states[i], states[i+1]);
            int steps = std::max(1, (int)std::ceil(L / stepLen));
            for (int k = 1; k <= steps; ++k) {
                double t = double(k) / steps;
                base::State *tmp = si_->allocState();
                si_->getStateSpace()->interpolate(states[i], states[i+1], t, tmp);

                toReals(tmp, now);
                for (size_t j = 0; j < J; ++j) {
                    const double dq = mask[j] ? shortestAngularDelta(prev[j], now[j])
                                            : (now[j] - prev[j]);
                    perJointUsage[j] += std::fabs(dq);
                }
                prev.swap(now);
                si_->freeState(tmp);
            }
        }


        // ============== data publishing ======================

        const double totalAngleUsage =
            std::accumulate(perJointUsage.begin(), perJointUsage.end(), 0.0);

        if (ros::isInitialized())
        {
            static ros::NodeHandle nh;
            static ros::Publisher pub =
                nh.advertise<std_msgs::Float64MultiArray>("/joint_usage", 10, true);

            std_msgs::Float64MultiArray msg;
            msg.data.reserve(1 + J);
            msg.data.push_back(totalAngleUsage);
            for (std::size_t j = 0; j < J; ++j)
                msg.data.push_back(perJointUsage[j]);

            pub.publish(msg);
            OMPL_INFORM("RULRRTstar: published joint usage, total=%.3f", totalAngleUsage);
        }

        // === RUL损失（逐关节 & 总）计算并发布 ===
        double totalLoss = 0.0;
        std::vector<double> perJointLoss(J, 0.0);

        if (auto *rulObj = dynamic_cast<ompl::base::RULAwareOptimizationObjective*>(opt_.get()))
        {
            const auto &RUL    = rulObj->getRUL();
            const auto &gamma  = rulObj->getGamma();
            const auto &mask   = rulObj->getRevoluteMask();
            const double alpha = rulObj->getAlpha();
            const double lambda= rulObj->getLambda();
            const double rmin  = rulObj->getRULmin();
            const double eps   = rulObj->getEpsilon();

            auto shortestAngularDelta = [](double a, double b) -> double {
                return std::remainder(b - a, 2.0 * M_PI); // (-π, π]
            };

            for (std::size_t i = 0; i + 1 < states.size(); ++i)
            {
                auto *s1 = states[i]->as<ompl::base::RealVectorStateSpace::StateType>();
                auto *s2 = states[i + 1]->as<ompl::base::RealVectorStateSpace::StateType>();

                for (std::size_t j = 0; j < J; ++j)
                {
                    const bool useSO2 = (j < mask.size()) ? mask[j] : false;
                    const double dq   = useSO2
                                    ? shortestAngularDelta((*s1)[j], (*s2)[j])
                                    : ((*s2)[j] - (*s1)[j]);

                    const double absdq = std::fabs(dq);
                    const double denom = std::max(RUL[j], rmin) + eps;
                    const double unit  = absdq / denom;                 // 路径长度（按寿命归一）
                    const double w     = alpha * unit + lambda * gamma[j] * unit;

                    perJointLoss[j] += w;
                    totalLoss       += w;
                }
            }
        }
        else
        {
            // 兜底：若不是RUL目标，就直接用路径几何增量累计（与opt_->motionCost不同，但避免崩）
            for (std::size_t i = 0; i + 1 < states.size(); ++i)
                totalLoss += si_->distance(states[i], states[i + 1]);
        }

        // 发布 /joint_loss  MultiArray: [totalLoss, loss_j0, loss_j1, ...]
        if (ros::isInitialized())
        {
            static ros::NodeHandle nh;
            static ros::Publisher pubLoss =
                nh.advertise<std_msgs::Float64MultiArray>("/joint_loss", 10, true);

            std_msgs::Float64MultiArray lossMsg;
            lossMsg.data.reserve(1 + J);
            lossMsg.data.push_back(totalLoss);
            for (std::size_t j = 0; j < J; ++j)
                lossMsg.data.push_back(perJointLoss[j]);

            pubLoss.publish(lossMsg);
            OMPL_INFORM("RULRRTstar: published joint loss, total=%.6f", totalLoss);
        }




        // 如果你只关注某个关节（比如低RUL电机），可以单独打印：
        // int jstar = 1; // 举例：关节1
        // OMPL_INFORM("[AngleUsage] focus joint j%d = %.6f rad", jstar, perJointUsage[jstar]);


        // Add the solution path.
        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());

        // If we don't have a goal motion, the solution is approximate
        if (!bestGoalMotion_)
            psol.setApproximate(approxDist);

        // Does the solution satisfy the optimization objective?
        psol.setOptimized(opt_, newSolution->cost, opt_->isSatisfied(bestCost_));
        pdef_->addSolutionPath(psol);
    }
    // No else, we have nothing

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree. Final solution cost "
                "%.3f",
                getName().c_str(), statesGenerated, rewireTest, goalMotions_.size(), bestCost_.value());

    // We've added a solution if newSolution == true, and it is an approximate solution if bestGoalMotion_ == false
    return base::PlannerStatus(newSolution != nullptr, bestGoalMotion_ == nullptr);
}


void ompl::geometric::RULRRTstar::getNeighbors(Motion *motion, std::vector<Motion *> &nbh) const
{
    auto cardDbl = static_cast<double>(nn_->size() + 1u);
    if (useKNearest_)
    {
        //- k-nearest RRT*
        unsigned int k = std::ceil(k_rrt_ * log(cardDbl));
        nn_->nearestK(motion, k, nbh);
    }
    else
    {
        double r = std::min(
            maxDistance_, r_rrt_ * std::pow(log(cardDbl) / cardDbl, 1 / static_cast<double>(si_->getStateDimension())));
        nn_->nearestR(motion, r, nbh);
    }
}

void ompl::geometric::RULRRTstar::removeFromParent(Motion *m)
{
    for (auto it = m->parent->children.begin(); it != m->parent->children.end(); ++it)
    {
        if (*it == m)
        {
            m->parent->children.erase(it);
            break;
        }
    }
}

void ompl::geometric::RULRRTstar::updateChildCosts(Motion *m)
{
    for (std::size_t i = 0; i < m->children.size(); ++i)
    {
        m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
        updateChildCosts(m->children[i]);
    }
}

void ompl::geometric::RULRRTstar::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void ompl::geometric::RULRRTstar::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (bestGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(bestGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

int ompl::geometric::RULRRTstar::pruneTree(const base::Cost &pruneTreeCost)
{
    // Variable
    // The percent improvement (expressed as a [0,1] fraction) in cost
    double fracBetter;
    // The number pruned
    int numPruned = 0;

    if (opt_->isFinite(prunedCost_))
    {
        fracBetter = std::abs((pruneTreeCost.value() - prunedCost_.value()) / prunedCost_.value());
    }
    else
    {
        fracBetter = 1.0;
    }

    if (fracBetter > pruneThreshold_)
    {
        // We are only pruning motions if they, AND all descendents, have a estimated cost greater than pruneTreeCost
        // The easiest way to do this is to find leaves that should be pruned and ascend up their ancestry until a
        // motion is found that is kept.
        // To avoid making an intermediate copy of the NN structure, we process the tree by descending down from the
        // start(s).
        // In the first pass, all Motions with a cost below pruneTreeCost, or Motion's with children with costs below
        // pruneTreeCost are added to the replacement NN structure,
        // while all other Motions are stored as either a 'leaf' or 'chain' Motion. After all the leaves are
        // disconnected and deleted, we check
        // if any of the the chain Motions are now leaves, and repeat that process until done.
        // This avoids (1) copying the NN structure into an intermediate variable and (2) the use of the expensive
        // NN::remove() method.

        // Variable
        // The queue of Motions to process:
        std::queue<Motion *, std::deque<Motion *>> motionQueue;
        // The list of leaves to prune
        std::queue<Motion *, std::deque<Motion *>> leavesToPrune;
        // The list of chain vertices to recheck after pruning
        std::list<Motion *> chainsToRecheck;

        // Clear the NN structure:
        nn_->clear();

        // Put all the starts into the NN structure and their children into the queue:
        // We do this so that start states are never pruned.
        for (auto &startMotion : startMotions_)
        {
            // Add to the NN
            nn_->add(startMotion);

            // Add their children to the queue:
            addChildrenToList(&motionQueue, startMotion);
        }

        while (motionQueue.empty() == false)
        {
            // Test, can the current motion ever provide a better solution?
            if (keepCondition(motionQueue.front(), pruneTreeCost))
            {
                // Yes it can, so it definitely won't be pruned
                // Add it back into the NN structure
                nn_->add(motionQueue.front());

                // Add it's children to the queue
                addChildrenToList(&motionQueue, motionQueue.front());
            }
            else
            {
                // No it can't, but does it have children?
                if (motionQueue.front()->children.empty() == false)
                {
                    // Yes it does.
                    // We can minimize the number of intermediate chain motions if we check their children
                    // If any of them won't be pruned, then this motion won't either. This intuitively seems
                    // like a nice balance between following the descendents forever.

                    // Variable
                    // Whether the children are definitely to be kept.
                    bool keepAChild = false;

                    // Find if any child is definitely not being pruned.
                    for (unsigned int i = 0u; keepAChild == false && i < motionQueue.front()->children.size(); ++i)
                    {
                        // Test if the child can ever provide a better solution
                        keepAChild = keepCondition(motionQueue.front()->children.at(i), pruneTreeCost);
                    }

                    // Are we *definitely* keeping any of the children?
                    if (keepAChild)
                    {
                        // Yes, we are, so we are not pruning this motion
                        // Add it back into the NN structure.
                        nn_->add(motionQueue.front());
                    }
                    else
                    {
                        // No, we aren't. This doesn't mean we won't though
                        // Move this Motion to the temporary list
                        chainsToRecheck.push_back(motionQueue.front());
                    }

                    // Either way. add it's children to the queue
                    addChildrenToList(&motionQueue, motionQueue.front());
                }
                else
                {
                    // No, so we will be pruning this motion:
                    leavesToPrune.push(motionQueue.front());
                }
            }

            // Pop the iterator, std::list::erase returns the next iterator
            motionQueue.pop();
        }

        // We now have a list of Motions to definitely remove, and a list of Motions to recheck
        // Iteratively check the two lists until there is nothing to to remove
        while (leavesToPrune.empty() == false)
        {
            // First empty the current leaves-to-prune
            while (leavesToPrune.empty() == false)
            {
                // If this leaf is a goal, remove it from the goal set
                if (leavesToPrune.front()->inGoal == true)
                {
                    // Warn if pruning the _best_ goal
                    if (leavesToPrune.front() == bestGoalMotion_)
                    {
                        OMPL_ERROR("%s: Pruning the best goal.", getName().c_str());
                    }
                    // Remove it
                    goalMotions_.erase(std::remove(goalMotions_.begin(), goalMotions_.end(), leavesToPrune.front()),
                                       goalMotions_.end());
                }

                // Remove the leaf from its parent
                removeFromParent(leavesToPrune.front());

                // Erase the actual motion
                // First free the state
                si_->freeState(leavesToPrune.front()->state);

                // then delete the pointer
                delete leavesToPrune.front();

                // And finally remove it from the list, erase returns the next iterator
                leavesToPrune.pop();

                // Update our counter
                ++numPruned;
            }

            // Now, we need to go through the list of chain vertices and see if any are now leaves
            auto mIter = chainsToRecheck.begin();
            while (mIter != chainsToRecheck.end())
            {
                // Is the Motion a leaf?
                if ((*mIter)->children.empty() == true)
                {
                    // It is, add to the removal queue
                    leavesToPrune.push(*mIter);

                    // Remove from this queue, getting the next
                    mIter = chainsToRecheck.erase(mIter);
                }
                else
                {
                    // Is isn't, skip to the next
                    ++mIter;
                }
            }
        }

        // Now finally add back any vertices left in chainsToReheck.
        // These are chain vertices that have descendents that we want to keep
        for (const auto &r : chainsToRecheck)
            // Add the motion back to the NN struct:
            nn_->add(r);

        // All done pruning.
        // Update the cost at which we've pruned:
        prunedCost_ = pruneTreeCost;

        // And if we're using the pruned measure, the measure to which we've pruned
        if (usePrunedMeasure_)
        {
            prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);

            if (useKNearest_ == false)
            {
                calculateRewiringLowerBounds();
            }
        }
        // No else, prunedMeasure_ is the si_ measure by default.
    }

    return numPruned;
}

void ompl::geometric::RULRRTstar::addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion)
{
    for (auto &child : motion->children)
    {
        motionList->push(child);
    }
}

bool ompl::geometric::RULRRTstar::keepCondition(const Motion *motion, const base::Cost &threshold) const
{
    // We keep if the cost-to-come-heuristic of motion is <= threshold, by checking
    // if !(threshold < heuristic), as if b is not better than a, then a is better than, or equal to, b
    if (bestGoalMotion_ && motion == bestGoalMotion_)
    {
        // If the threshold is the theoretical minimum, the bestGoalMotion_ will sometimes fail the test due to floating point precision. Avoid that.
        return true;
    }

    return !opt_->isCostBetterThan(threshold, solutionHeuristic(motion));
}

ompl::base::Cost ompl::geometric::RULRRTstar::solutionHeuristic(const Motion *motion) const
{
    base::Cost costToCome;
    if (useAdmissibleCostToCome_)
    {
        // Start with infinite cost
        costToCome = opt_->infiniteCost();

        // Find the min from each start
        for (auto &startMotion : startMotions_)
        {
            costToCome = opt_->betterCost(
                costToCome, opt_->motionCost(startMotion->state,
                                             motion->state));  // lower-bounding cost from the start to the state
        }
    }
    else
    {
        costToCome = motion->cost;  // current cost from the state to the goal
    }

    const base::Cost costToGo =
        opt_->costToGo(motion->state, pdef_->getGoal().get());  // lower-bounding cost from the state to the goal
    return opt_->combineCosts(costToCome, costToGo);            // add the two costs
}

void ompl::geometric::RULRRTstar::setTreePruning(const bool prune)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // If we just disabled tree pruning, but we wee using prunedMeasure, we need to disable that as it required myself
    if (prune == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Store
    useTreePruning_ = prune;
}

void ompl::geometric::RULRRTstar::setPrunedMeasure(bool informedMeasure)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option only works with informed sampling
    if (informedMeasure == true && (useInformedSampling_ == false || useTreePruning_ == false))
    {
        OMPL_ERROR("%s: InformedMeasure requires InformedSampling and TreePruning.", getName().c_str());
    }

    // Check if we're changed and update parameters if we have:
    if (informedMeasure != usePrunedMeasure_)
    {
        // Store the setting
        usePrunedMeasure_ = informedMeasure;

        // Update the prunedMeasure_ appropriately, if it has been configured.
        if (setup_ == true)
        {
            if (usePrunedMeasure_)
            {
                prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);
            }
            else
            {
                prunedMeasure_ = si_->getSpaceMeasure();
            }
        }

        // And either way, update the rewiring radius if necessary
        if (useKNearest_ == false)
        {
            calculateRewiringLowerBounds();
        }
    }
}

void ompl::geometric::RULRRTstar::setInformedSampling(bool informedSampling)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setSampleRejection, assert that:
    if (informedSampling == true && useRejectionSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // If we just disabled tree pruning, but we are using prunedMeasure, we need to disable that as it required myself
    if (informedSampling == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Check if we're changing the setting of informed sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (informedSampling != useInformedSampling_)
    {
        // If we're disabled informedSampling, and prunedMeasure is enabled, we need to disable that
        if (informedSampling == false && usePrunedMeasure_ == true)
        {
            setPrunedMeasure(false);
        }

        // Store the value
        useInformedSampling_ = informedSampling;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::RULRRTstar::setSampleRejection(const bool reject)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setInformedSampling, assert that:
    if (reject == true && useInformedSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // Check if we're changing the setting of rejection sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (reject != useRejectionSampling_)
    {
        // Store the setting
        useRejectionSampling_ = reject;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::RULRRTstar::setOrderedSampling(bool orderSamples)
{
    // Make sure we're using some type of informed sampling
    if (useInformedSampling_ == false && useRejectionSampling_ == false)
    {
        OMPL_ERROR("%s: OrderedSampling requires either informed sampling or rejection sampling.", getName().c_str());
    }

    // Check if we're changing the setting. If we are, we will need to create a new sampler, which we only want to do if
    // one is already allocated.
    if (orderSamples != useOrderedSampling_)
    {
        // Store the setting
        useOrderedSampling_ = orderSamples;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::RULRRTstar::allocSampler()
{
    // Allocate the appropriate type of sampler.
    if (useInformedSampling_)
    {
        // We are using informed sampling, this can end-up reverting to rejection sampling in some cases
        OMPL_INFORM("%s: Using informed sampling.", getName().c_str());
        infSampler_ = opt_->allocInformedStateSampler(pdef_, numSampleAttempts_);
    }
    else if (useRejectionSampling_)
    {
        // We are explicitly using rejection sampling.
        OMPL_INFORM("%s: Using rejection sampling.", getName().c_str());
        infSampler_ = std::make_shared<base::RejectionInfSampler>(pdef_, numSampleAttempts_);
    }
    else
    {
        // We are using a regular sampler
        sampler_ = si_->allocStateSampler();
    }

    // Wrap into a sorted sampler
    if (useOrderedSampling_ == true)
    {
        infSampler_ = std::make_shared<base::OrderedInfSampler>(infSampler_, batchSize_);
    }
    // No else
}

bool ompl::geometric::RULRRTstar::sampleUniform(base::State *statePtr)
{
    // Use the appropriate sampler
    if (useInformedSampling_ || useRejectionSampling_)
    {
        // Attempt the focused sampler and return the result.
        // If bestCost is changing a lot by small amounts, this could
        // be prunedCost_ to reduce the number of times the informed sampling
        // transforms are recalculated.
        return infSampler_->sampleUniform(statePtr, bestCost_);
    }
    else
    {
        // Simply return a state from the regular sampler
        sampler_->sampleUniform(statePtr);

        // Always true
        return true;
    }
}

void ompl::geometric::RULRRTstar::calculateRewiringLowerBounds()
{
    const auto dimDbl = static_cast<double>(si_->getStateDimension());

    // k_rrt > 2^(d + 1) * e * (1 + 1 / d).  K-nearest RRT*
    k_rrt_ = rewireFactor_ * (std::pow(2, dimDbl + 1) * boost::math::constants::e<double>() * (1.0 + 1.0 / dimDbl));

    // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
    // If we're not using the informed measure, prunedMeasure_ will be set to si_->getSpaceMeasure();
    r_rrt_ =
        rewireFactor_ *
        std::pow(2 * (1.0 + 1.0 / dimDbl) * (prunedMeasure_ / unitNBallMeasure(si_->getStateDimension())), 1.0 / dimDbl);
}

double ompl::geometric::RULRRTstar::rulAwareDistance(const Motion* A, const Motion* B) const
{
    const std::size_t J = si_->getStateDimension();

    // 如果不是 RUL-aware 目标，退回到默认的几何距离
    auto rul = std::dynamic_pointer_cast<ompl::base::RULAwareOptimizationObjective>(opt_);
    if (!rul)
        return si_->distance(A->state, B->state);

    const auto& mask   = rul->getRevoluteMask();
    const auto& RUL    = rul->getRUL();
    const auto& gamma  = rul->getGamma();
    const double alpha = rul->getAlpha();
    const double lambda= rul->getLambda();
    const double rmin  = rul->getRULmin();
    const double eps   = rul->getEpsilon();

    const auto* q1 = A->state->as<ompl::base::RealVectorStateSpace::StateType>();
    const auto* q2 = B->state->as<ompl::base::RealVectorStateSpace::StateType>();

    auto ang = [](double a, double b){ return std::remainder(b - a, 2.0 * M_PI); };

    double d = 0.0;
    for (std::size_t i = 0; i < J; ++i)
    {
        const bool useSO2 = (i < mask.size()) ? mask[i] : false;
        const double dq   = useSO2 ? ang(q1->values[i], q2->values[i])
                                   : (q2->values[i] - q1->values[i]);

        // 权重：结合 alpha、lambda、gamma、RUL
        const double wi = (alpha + lambda * gamma[i]) / (std::max(RUL[i], rmin) + eps);
        d += std::fabs(dq) * wi;
    }
    return d;
}

// double ompl::geometric::RULRRTstar::rulAwareDistance(const Motion* A, const Motion* B) const
// {
//     // 若不是 RUL-aware 目标，退回默认几何距离
//     auto rul = std::dynamic_pointer_cast<ompl::base::RULAwareOptimizationObjective>(opt_);
//     if (!rul)
//         return si_->distance(A->state, B->state);

//     // 读取参数（与 motionCost 一致）
//     const auto& mask    = rul->getRevoluteMask();
//     const auto& RUL     = rul->getRUL();
//     const auto& gamma   = rul->getGamma();
//     const double alpha  = rul->getAlpha();
//     const double lambda = rul->getLambda();
//     const double eps    = rul->getEpsilon();

//     // 取状态为实数向量，兼容 CompoundStateSpace
//     std::vector<double> v1, v2;
//     si_->getStateSpace()->copyToReals(v1, A->state);
//     si_->getStateSpace()->copyToReals(v2, B->state);
//     const std::size_t J = v1.size();

//     auto shortest = [](double a, double b){ return std::remainder(b - a, 2.0 * M_PI); };

//     double sum = 0.0;
//     for (std::size_t i = 0; i < J; ++i)
//     {
//         const bool wrap = (i < mask.size()) ? mask[i] : false;
//         const double dq = wrap ? shortest(v1[i], v2[i]) : (v2[i] - v1[i]);
//         const double absdq = std::fabs(dq);

//         // fixed1000 归一化 + 下限 0.05（与 motionCost 完全一致）
//         double rul_norm = RUL[i] / 1000.0;
//         if (rul_norm < 0.05) rul_norm = 0.05;

//         // 等价展开：|dq| * [ α + λ * γ / (rul_norm + ε) ]
//         const double weight = alpha + lambda * (gamma[i] / (rul_norm + eps));
//         sum += absdq * weight;
//     }
//     return sum;
// }

