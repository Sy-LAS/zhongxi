#include "slam2d/back_end.h"
#include <iostream>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <memory>

namespace slam2d {

// 定义求解器类型
typedef g2o::BlockSolver< g2o::BlockSolverTraits<2, 1> >  SlamBlockSolver;
typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>  LinearSolverEigen;

BackEnd::BackEnd() : opt_params_() {}

G2OBackend::G2OBackend() : current_vertex_id_(0) {
    // 初始化g2o优化器
    optimizer_ = new g2o::SparseOptimizer();
    
    // 设置线性求解器
    auto linearSolver = std::make_unique<LinearSolverEigen>();
    linearSolver->setBlockOrdering(false);
    auto blockSolver = std::make_unique<SlamBlockSolver>(std::move(linearSolver));
    
    // 选择优化算法 (Levenberg-Marquardt)
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
    optimizer_->setAlgorithm(algorithm);
    
    // 设置终止条件
    optimizer_->setVerbose(false);  // 可以设为true查看优化过程
}

G2OBackend::~G2OBackend() {
    // 清理g2o优化器资源
    if (optimizer_) {
        delete optimizer_;
        optimizer_ = nullptr;
    }
}

bool G2OBackend::addPoseNode(const Pose2D& pose) {
    // 创建顶点 (位姿节点)
    g2o::VertexSE2* vertex = new g2o::VertexSE2();
    vertex->setId(current_vertex_id_);
    
    // 设置初始估计值
    g2o::SE2 se2_pose(pose.x, pose.y, pose.theta);
    vertex->setEstimate(se2_pose);
    
    // 第一个节点为固定节点，作为参考
    if (current_vertex_id_ == 0) {
        vertex->setFixed(true);
    }
    
    // 添加到优化器
    optimizer_->addVertex(vertex);
    
    // 保存顶点ID与位姿的映射关系
    pose_vertices_[current_vertex_id_] = vertex;
    
    current_vertex_id_++;
    return true;
}

bool G2OBackend::addConstraint(const Constraint2D& constraint) {
    // 创建边 (约束)
    g2o::EdgeSE2* edge = new g2o::EdgeSE2();
    
    // 设置连接的顶点
    edge->vertices()[0] = optimizer_->vertex(constraint.from_id);
    edge->vertices()[1] = optimizer_->vertex(constraint.to_id);
    
    // 设置测量值 (相对位姿)
    g2o::SE2 measurement(constraint.relative_pose.x, 
                        constraint.relative_pose.y, 
                        constraint.relative_pose.theta);
    edge->setMeasurement(measurement);
    
    // 设置信息矩阵 (协方差矩阵的逆)
    Eigen::Matrix3d information_matrix = Eigen::Matrix3d::Zero();
    information_matrix(0, 0) = constraint.information(0, 0);  // x-x
    information_matrix(1, 1) = constraint.information(1, 1);  // y-y
    information_matrix(2, 2) = constraint.information(2, 2);  // theta-theta
    information_matrix(0, 1) = constraint.information(0, 1);  // x-y
    information_matrix(1, 0) = constraint.information(1, 0);  // y-x
    information_matrix(0, 2) = constraint.information(0, 2);  // x-theta
    information_matrix(2, 0) = constraint.information(2, 0);  // theta-x
    information_matrix(1, 2) = constraint.information(1, 2);  // y-theta
    information_matrix(2, 1) = constraint.information(2, 1);  // theta-y
    
    edge->setInformation(information_matrix);
    
    // 添加到优化器
    optimizer_->addEdge(edge);
    
    return true;
}

bool G2OBackend::optimize() {
    // 执行优化
    optimizer_->initializeOptimization();
    int num_iters = optimizer_->optimize(opt_params_.max_iterations);
    
    std::cout << "Performed " << num_iters << " optimization iterations" << std::endl;
    
    return num_iters > 0;
}

bool G2OBackend::getOptimizedPose(int id, Pose2D& pose) const {
    // 从优化器获取优化后的位姿
    g2o::VertexSE2* vertex = dynamic_cast<g2o::VertexSE2*>(optimizer_->vertex(id));
    if (!vertex) {
        std::cerr << "Error: Could not find vertex with id " << id << std::endl;
        return false;
    }
    
    g2o::SE2 optimized_se2 = vertex->estimate();
    pose.x = optimized_se2.translation().x();
    pose.y = optimized_se2.translation().y();
    pose.theta = optimized_se2.rotation().angle();
    
    return true;
}

std::vector<Pose2D> G2OBackend::getOptimizedTrajectory() const {
    // 获取整个优化后的轨迹
    std::vector<Pose2D> trajectory;
    
    // 遍历所有顶点，按ID排序
    std::vector<int> vertex_ids;
    for (const auto& pair : pose_vertices_) {
        vertex_ids.push_back(pair.first);
    }
    std::sort(vertex_ids.begin(), vertex_ids.end());
    
    for (int id : vertex_ids) {
        Pose2D pose;
        if (getOptimizedPose(id, pose)) {
            trajectory.push_back(pose);
        } else {
            std::cerr << "Warning: Could not retrieve pose for vertex " << id << std::endl;
        }
    }
    
    return trajectory;
}

} // namespace slam2d