#include "slam2d/map.h"
#include "slam2d/utils.h"
#include "slam2d/utils/transformations.h"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace slam2d {

double SLAMUtils::poseError(const Pose2D& pose1, const Pose2D& pose2) {
    double dx = pose1.x - pose2.x;
    double dy = pose1.y - pose2.y;
    double dtheta = utils::normalizeAngle(pose1.theta - pose2.theta);
    
    return std::sqrt(dx*dx + dy*dy + dtheta*dtheta);
}

Eigen::Matrix3d SLAMUtils::computeJacobian(const Pose2D& from, const Pose2D& to, const Pose2D& z) {
    Eigen::Matrix3d J;
    
    // 计算从from到to的期望测量值
    Pose2D expected_meas = utils::relativePose(from, to);
    
    // 计算误差
    double dx = z.x - expected_meas.x;
    double dy = z.y - expected_meas.y;
    double dtheta = utils::normalizeAngle(z.theta - expected_meas.theta);
    
    // 简化的雅可比矩阵计算
    J << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;
    
    return J;
}

int SLAMUtils::dataAssociation(const Point2D& observed_point, 
                              const std::vector<Point2D>& landmarks, 
                              double threshold) {
    if (landmarks.empty()) {
        return -1;
    }
    
    int closest_idx = -1;
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < landmarks.size(); ++i) {
        double dist = utils::distance(observed_point, landmarks[i]);
        if (dist < min_dist && dist < threshold) {
            min_dist = dist;
            closest_idx = static_cast<int>(i);
        }
    }
    
    return closest_idx;
}

Covariance2D SLAMUtils::computeInformationMatrix(const Covariance2D& covariance) {
    // 计算协方差矩阵的逆（信息矩阵）
    Eigen::Matrix3d cov_mat = covariance;
    Eigen::Matrix3d info_mat = cov_mat.inverse();
    
    return info_mat;
}

std::vector<std::pair<Point2D, Point2D>> SLAMUtils::extractLinesFromScan(const LaserScan& scan) {
    std::vector<std::pair<Point2D, Point2D>> lines;
    
    if (scan.ranges.empty()) {
        return lines;
    }
    
    // 将极坐标转换为直角坐标
    std::vector<Point2D> points;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        if (scan.ranges[i] > scan.range_min && scan.ranges[i] < scan.range_max) {
            Point2D pt = utils::polarToCartesian(scan.ranges[i], scan.angles[i]);
            points.push_back(pt);
        }
    }
    
    // 简单的直线提取算法（相邻点斜率相似则认为在同一直线上）
    if (points.size() < 2) {
        return lines;
    }
    
    // 在实际应用中，这里可以使用更复杂的算法，如RANSAC
    for (size_t i = 0; i < points.size() - 1; ++i) {
        // 将相邻的点对作为线段
        lines.emplace_back(points[i], points[i+1]);
    }
    
    return lines;
}

bool SLAMUtils::loopDetection(const Pose2D& current_pose, 
                            const std::vector<Pose2D>& candidate_poses,
                            double threshold) {
    for (const auto& pose : candidate_poses) {
        double dist = utils::poseDistance(current_pose, pose);
        if (dist < threshold) {
            // 进一步检查角度差异
            double angle_diff = std::abs(utils::normalizeAngle(current_pose.theta - pose.theta));
            if (angle_diff < M_PI / 4) {  // 角度差异小于45度
                return true;
            }
        }
    }
    return false;
}

double SLAMUtils::calculatePathLength(const std::vector<Pose2D>& trajectory) {
    if (trajectory.size() < 2) {
        return 0.0;
    }
    
    double total_length = 0.0;
    for (size_t i = 1; i < trajectory.size(); ++i) {
        double dx = trajectory[i].x - trajectory[i-1].x;
        double dy = trajectory[i].y - trajectory[i-1].y;
        total_length += std::sqrt(dx*dx + dy*dy);
    }
    
    return total_length;
}

std::vector<Pose2D> SLAMUtils::smoothTrajectory(const std::vector<Pose2D>& trajectory, 
                                               int window_size) {
    if (trajectory.empty() || window_size <= 1) {
        return trajectory;
    }
    
    std::vector<Pose2D> smoothed_trajectory = trajectory;
    
    for (size_t i = 0; i < trajectory.size(); ++i) {
        int start_idx = std::max(0, static_cast<int>(i) - window_size/2);
        int end_idx = std::min(static_cast<int>(trajectory.size()-1), 
                               static_cast<int>(i) + window_size/2);
        
        double sum_x = 0.0, sum_y = 0.0, sum_theta = 0.0;
        int count = 0;
        
        for (int j = start_idx; j <= end_idx; ++j) {
            sum_x += trajectory[j].x;
            sum_y += trajectory[j].y;
            sum_theta += trajectory[j].theta;
            count++;
        }
        
        if (count > 0) {
            smoothed_trajectory[i].x = sum_x / count;
            smoothed_trajectory[i].y = sum_y / count;
            smoothed_trajectory[i].theta = utils::normalizeAngle(sum_theta / count);
        }
    }
    
    return smoothed_trajectory;
}

double SLAMUtils::evaluateMapQuality(const std::vector<GridCell>& map, 
                                   int width, int height) {
    if (map.empty()) {
        return 0.0;
    }
    
    int total_cells = width * height;
    int occupied_cells = 0;
    int unknown_cells = 0;
    
    for (const auto& cell : map) {
        if (cell.occupancy_prob > 0.7) {
            occupied_cells++;
        } else if (std::abs(cell.occupancy_prob - 0.5) < 0.01) {
            unknown_cells++;
        }
    }
    
    // 地图质量评估：已探索区域比例 + 占用信息丰富度
    double exploration_ratio = static_cast<double>(total_cells - unknown_cells) / total_cells;
    double occupancy_ratio = static_cast<double>(occupied_cells) / (total_cells - unknown_cells + 1e-6);
    
    // 归一化到[0,1]范围
    return exploration_ratio * 0.6 + std::min(1.0, occupancy_ratio) * 0.4;
}
double SLAMUtils::sampleNormalDistribution(double mean, double stddev) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::normal_distribution<> dist(mean, stddev);
    return dist(gen);
} // namespace slam2d#include <random>


}
