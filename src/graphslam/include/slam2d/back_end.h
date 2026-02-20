#ifndef BACK_END_H
#define BACK_END_H

#include <vector>
#include <memory>
#include <functional>
#include "types.h"
#include "map.h"

namespace slam2d {

// 优化参数
struct OptimizationParams {
    int max_iterations = 100;      // 最大迭代次数
    double lambda_init = 1e-3;     // Levenberg-Marquardt算法初始lambda
    double lambda_factor = 10.0;   // LM算法lambda因子
    double epsilon = 1e-6;         // 收敛阈值
    
    OptimizationParams() = default;
    OptimizationParams(int max_iter, double lam_init, double lam_fact, double eps)
        : max_iterations(max_iter), lambda_init(lam_init), 
          lambda_factor(lam_fact), epsilon(eps) {}
};

// 后端优化类
class BackEnd {
public:
    BackEnd();
    virtual ~BackEnd() = default;

    // 添加位姿节点
    virtual bool addPoseNode(const Pose2D& pose) = 0;

    // 添加约束
    virtual bool addConstraint(const Constraint2D& constraint) = 0;

    // 执行优化
    virtual bool optimize() = 0;

    // 获取优化后的位姿
    virtual bool getOptimizedPose(int id, Pose2D& pose) const = 0;

    // 设置优化参数
    void setOptimizationParams(const OptimizationParams& params) { 
        opt_params_ = params; 
    }

    // 设置地图指针
    void setMap(MapPtr map) { map_ = map; }

    // 获取优化后的轨迹
    virtual std::vector<Pose2D> getOptimizedTrajectory() const = 0;

protected:
    OptimizationParams opt_params_;
    MapPtr map_;
};

// 基于g2o的后端实现
class G2OBackend : public BackEnd {
public:
    G2OBackend();
    virtual ~G2OBackend();

    // 添加位姿节点
    bool addPoseNode(const Pose2D& pose) override;

    // 添加约束
    bool addConstraint(const Constraint2D& constraint) override;

    // 执行优化
    bool optimize() override;

    // 获取优化后的位姿
    bool getOptimizedPose(int id, Pose2D& pose) const override;

    // 获取优化后的轨迹
    std::vector<Pose2D> getOptimizedTrajectory() const override;

private:
    void* optimizer_;  // g2o优化器指针（在实现中具体定义）
    int current_vertex_id_;
};

using BackEndPtr = std::shared_ptr<BackEnd>;

} // namespace slam2d

#endif // BACK_END_H