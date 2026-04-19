#include "slam2d/front_end.h"
#include "slam2d/utils/transformations.h"
#include "slam2d/utils/laser_scan.h"
#include <algorithm>
#include <limits>
#include <cmath>
#include <Eigen/Dense>

namespace slam2d {

FrontEnd::FrontEnd() : odometry_params_() {}

Pose2D FrontEnd::predictPose(const Pose2D& prev_pose, const Pose2D& control) {
    // 使用运动模型预测下一个位姿
    // 这里使用简单的差速驱动模型
    Pose2D predicted_pose;
    
    double rot1 = std::atan2(control.y, control.x);
    double trans = std::sqrt(control.x * control.x + control.y * control.y);
    double rot2 = control.theta - rot1;
    
    // 添加运动不确定性
    double delta_rot1 = rot1 + utils::sampleNormalDistribution(
        0, odometry_params_.alpha1 * rot1 * rot1 + 
           odometry_params_.alpha2 * trans * trans);
    double delta_trans = trans + utils::sampleNormalDistribution(
        0, odometry_params_.alpha3 * trans * trans + 
           odometry_params_.alpha4 * rot1 * rot1 + 
           odometry_params_.alpha4 * rot2 * rot2);
    double delta_rot2 = rot2 + utils::sampleNormalDistribution(
        0, odometry_params_.alpha1 * rot2 * rot2 + 
           odometry_params_.alpha2 * trans * trans);
    
    predicted_pose.x = prev_pose.x + delta_trans * std::cos(prev_pose.theta + delta_rot1);
    predicted_pose.y = prev_pose.y + delta_trans * std::sin(prev_pose.theta + delta_rot1);
    predicted_pose.theta = prev_pose.theta + delta_rot1 + delta_rot2;
    
    predicted_pose.theta = utils::normalizeAngle(predicted_pose.theta);
    
    return predicted_pose;
}

ICPFrontEnd::ICPFrontEnd() : max_iterations_(50), convergence_threshold_(1e-6) {}

bool ICPFrontEnd::processLaserScan(const LaserScan& scan, const Pose2D& predicted_pose, 
                                  Pose2D& corrected_pose, std::vector<Observation>& observations) {
    // 如果是第一帧，直接使用预测位姿
    if (last_scan_.ranges.empty()) {
        last_scan_ = scan;
        last_pose_ = predicted_pose;
        corrected_pose = predicted_pose;
        
        // 从扫描数据中提取观测
        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            if (scan.ranges[i] > scan.range_min && scan.ranges[i] < scan.range_max) {
                Point2D local_point = utils::polarToCartesian(scan.ranges[i], scan.angles[i]);
                observations.emplace_back(local_point, scan.ranges[i], scan.angles[i], -1);
            }
        }
        
        return true;
    }
    
    // 执行ICP扫描匹配
    corrected_pose = scanMatchPose(scan, last_scan_, predicted_pose);
    
    // 更新内部状态
    last_scan_ = scan;
    last_pose_ = corrected_pose;
    
    // 从扫描数据中提取观测
    std::vector<Point2D> global_points = utils::LaserScanProcessor::scanToGlobal(scan, corrected_pose);
    for (size_t i = 0; i < global_points.size(); ++i) {
        if (scan.ranges[i] > scan.range_min && scan.ranges[i] < scan.range_max) {
            observations.emplace_back(global_points[i], scan.ranges[i], scan.angles[i], static_cast<int>(i));
        }
    }
    
    // 调用回调函数
    if (pose_callback_) {
        pose_callback_(corrected_pose);
    }
    
    return true;
}

Pose2D ICPFrontEnd::scanMatchPose(const LaserScan& current_scan, 
                                 const LaserScan& prev_scan, 
                                 const Pose2D& initial_pose) {
    Pose2D transform = Pose2D(0, 0, 0);
    
    for (int iter = 0; iter < max_iterations_; ++iter) {
        // 找到最近点对应关系
        std::vector<std::pair<Point2D, Point2D>> correspondences = 
            findClosestPoints(current_scan, prev_scan, initial_pose, transform);
        
        if (correspondences.size() < 10) {
            // 如果对应点太少，返回初始预测位姿
            return initial_pose;
        }
        
        // 计算最优变换
        Pose2D new_transform = computeTransform(correspondences);
        
        // 检查收敛性
        double dx = new_transform.x - transform.x;
        double dy = new_transform.y - transform.y;
        double dtheta = new_transform.theta - transform.theta;
        dtheta = utils::normalizeAngle(dtheta);
        
        if (dx*dx + dy*dy < convergence_threshold_*convergence_threshold_ &&
            std::abs(dtheta) < convergence_threshold_) {
            break;
        }
        
        transform = new_transform;
    }
    
    // 将局部变换应用到初始位姿上
    Pose2D result = composePoses(initial_pose, transform);
    return result;
}

std::vector<std::pair<Point2D, Point2D>> 
ICPFrontEnd::findClosestPoints(const LaserScan& current_scan, 
                              const LaserScan& prev_scan, 
                              const Pose2D& initial_pose, 
                              const Pose2D& transform) {
    std::vector<std::pair<Point2D, Point2D>> correspondences;
    
    // 将当前扫描变换到之前扫描的坐标系中
    Pose2D composed_pose = composePoses(initial_pose, transform);
    std::vector<Point2D> current_points = utils::LaserScanProcessor::scanToGlobal(current_scan, composed_pose);
    
    // 将上一次扫描转换回当前坐标系
    std::vector<Point2D> prev_points = utils::LaserScanProcessor::scanToGlobal(prev_scan, last_pose_);
    
    // 为当前扫描中的每个点找到上一次扫描中最接近的点
    for (const auto& curr_pt : current_points) {
        if (!std::isfinite(curr_pt.x) || !std::isfinite(curr_pt.y)) continue;
        
        double min_dist = std::numeric_limits<double>::max();
        Point2D closest_pt = prev_points[0]; // 默认值
        
        for (const auto& prev_pt : prev_points) {
            if (!std::isfinite(prev_pt.x) || !std::isfinite(prev_pt.y)) continue;
            
            // 手动计算欧几里得距离
            double dist = std::sqrt((curr_pt.x - prev_pt.x)*(curr_pt.x - prev_pt.x) + 
                                   (curr_pt.y - prev_pt.y)*(curr_pt.y - prev_pt.y));
            if (dist < min_dist) {
                min_dist = dist;
                if (min_dist < 0.5) { // 只考虑距离小于0.5米的点作为对应点
                    closest_pt = prev_pt;
                }
            }
        }
        
        if (min_dist < 0.5) { // 只保留合理的对应点
            correspondences.emplace_back(curr_pt, closest_pt);
        }
    }
    
    return correspondences;
}

Pose2D ICPFrontEnd::computeTransform(const std::vector<std::pair<Point2D, Point2D>>& correspondences) {
    if (correspondences.empty()) {
        return Pose2D(0, 0, 0);
    }
    
    // 计算质心
    Point2D curr_centroid(0, 0);
    Point2D prev_centroid(0, 0);
    
    for (const auto& corr : correspondences) {
        curr_centroid.x += corr.first.x;
        curr_centroid.y += corr.first.y;
        prev_centroid.x += corr.second.x;
        prev_centroid.y += corr.second.y;
    }
    
    curr_centroid.x /= correspondences.size();
    curr_centroid.y /= correspondences.size();
    prev_centroid.x /= correspondences.size();
    prev_centroid.y /= correspondences.size();
    
    // 计算协方差矩阵
    double H_xx = 0, H_xy = 0;
    double H_yx = 0, H_yy = 0;
    
    for (const auto& corr : correspondences) {
        Point2D centered_curr(corr.first.x - curr_centroid.x, 
                             corr.first.y - curr_centroid.y);
        Point2D centered_prev(corr.second.x - prev_centroid.x, 
                             corr.second.y - prev_centroid.y);
        
        H_xx += centered_curr.x * centered_prev.x;
        H_xy += centered_curr.x * centered_prev.y;
        H_yx += centered_curr.y * centered_prev.x;
        H_yy += centered_curr.y * centered_prev.y;
    }
    
    // SVD分解
    Eigen::Matrix2d H;
    H << H_xx, H_xy,
         H_yx, H_yy;
    
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();
    
    // 计算旋转矩阵
    Eigen::Matrix2d R = U * V.transpose();
    // 确保是旋转矩阵而不是反射矩阵
    if (R.determinant() < 0) {
        Eigen::Matrix2d V_prime = V;
        V_prime.col(1) *= -1;
        R = U * V_prime.transpose();
    }
    
    // 计算角度
    double theta = std::atan2(R(1, 0), R(0, 0));
    
    // 计算平移
    Eigen::Vector2d t_prev_centroid(prev_centroid.x, prev_centroid.y);
    Eigen::Vector2d t_curr_centroid(curr_centroid.x, curr_centroid.y);
    Eigen::Vector2d translation = t_prev_centroid - R * t_curr_centroid;
    
    return Pose2D(translation(0), translation(1), theta);
}

Pose2D ICPFrontEnd::composePoses(const Pose2D& p1, const Pose2D& p2) {
    double cos_theta = std::cos(p1.theta);
    double sin_theta = std::sin(p1.theta);
    
    Pose2D result;
    result.x = p1.x + cos_theta * p2.x - sin_theta * p2.y;
    result.y = p1.y + sin_theta * p2.x + cos_theta * p2.y;
    result.theta = p1.theta + p2.theta;
    result.theta = utils::normalizeAngle(result.theta);
    
    return result;
}

FeatureBasedFrontEnd::FeatureBasedFrontEnd() : feature_extraction_threshold_(0.1) {}

bool FeatureBasedFrontEnd::processLaserScan(const LaserScan& scan, const Pose2D& predicted_pose, 
                                          Pose2D& corrected_pose, std::vector<Observation>& observations) {
    // 提取特征点
    std::vector<Point2D> current_features = extractFeatures(scan);
    
    if (last_features_.empty()) {
        last_features_ = current_features;
        last_pose_ = predicted_pose;
        corrected_pose = predicted_pose;
        
        // 从扫描数据中提取观测
        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            if (scan.ranges[i] > scan.range_min && scan.ranges[i] < scan.range_max) {
                Point2D local_point = utils::polarToCartesian(scan.ranges[i], scan.angles[i]);
                Point2D global_point = utils::transformPoint(predicted_pose, local_point);
                observations.emplace_back(global_point, scan.ranges[i], scan.angles[i], -1);
            }
        }
        
        return true;
    }
    
    // 执行基于特征的扫描匹配
    corrected_pose = scanMatchPose(scan, LaserScan{}, predicted_pose);
    
    // 更新内部状态
    last_features_ = current_features;
    last_pose_ = corrected_pose;
    
    // 从扫描数据中提取观测
    std::vector<Point2D> global_points = utils::LaserScanProcessor::scanToGlobal(scan, corrected_pose);
    for (size_t i = 0; i < global_points.size(); ++i) {
        if (scan.ranges[i] > scan.range_min && scan.ranges[i] < scan.range_max) {
            observations.emplace_back(global_points[i], scan.ranges[i], scan.angles[i], static_cast<int>(i));
        }
    }
    
    // 调用回调函数
    if (pose_callback_) {
        pose_callback_(corrected_pose);
    }
    
    return true;
}

std::vector<Point2D> FeatureBasedFrontEnd::extractFeatures(const LaserScan& scan) {
    std::vector<Point2D> features;
    
    // 简单的边缘检测：寻找距离变化较大的点
    for (size_t i = 1; i < scan.ranges.size() - 1; ++i) {
        if (!std::isfinite(scan.ranges[i])) continue;
        
        double diff_prev = std::abs(scan.ranges[i] - scan.ranges[i-1]);
        double diff_next = std::abs(scan.ranges[i+1] - scan.ranges[i]);
        
        if (diff_prev > feature_extraction_threshold_ || 
            diff_next > feature_extraction_threshold_) {
            Point2D feature_point = utils::polarToCartesian(scan.ranges[i], scan.angles[i]);
            features.push_back(feature_point);
        }
    }
    
    return features;
}

Pose2D FeatureBasedFrontEnd::scanMatchPose(const LaserScan& current_scan, 
                                         const LaserScan& prev_scan, 
                                         const Pose2D& initial_pose) {
    // 基于特征的匹配实现
    // 这里简化处理，直接返回初始位姿
    return initial_pose;
}

} // namespace slam2d