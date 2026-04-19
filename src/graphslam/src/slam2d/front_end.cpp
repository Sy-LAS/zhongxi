#include "slam2d/front_end.h"
#include "utils/transformations.h"
#include "utils/laser_scan.h"
#include <algorithm>
#include <limits>
#include <cmath>

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
        observations.emplace_back(global_points[i], scan.ranges[i], scan.angles[i], static_cast<int>(i));
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
    Pose2D current_pose = initial_pose;
    std::vector<Point2D> curr_points = utils::LaserScanProcessor::polarToCartesianScan(current_scan);
    std::vector<Point2D> prev_points = utils::LaserScanProcessor::polarToCartesianScan(prev_scan);
    
    for (int iter = 0; iter < max_iterations_; ++iter) {
        // 计算当前位姿下的点集
        std::vector<Point2D> transformed_curr_points;
        for (const auto& pt : curr_points) {
            transformed_curr_points.push_back(utils::transformPoint(current_pose, pt));
        }
        
        // 寻找最近点对应关系
        std::vector<std::pair<Point2D, Point2D>> correspondences;
        for (const auto& curr_pt : transformed_curr_points) {
            double min_dist = std::numeric_limits<double>::max();
            Point2D closest_prev_pt;
            
            for (const auto& prev_pt : prev_points) {
                double dist = utils::distance(curr_pt, prev_pt);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_prev_pt = prev_pt;
                }
            }
            
            if (min_dist < 0.5) { // 距离阈值
                correspondences.emplace_back(curr_pt, closest_prev_pt);
            }
        }
        
        if (correspondences.empty()) {
            break;
        }
        
        // 计算变换
        double sum_x1 = 0, sum_y1 = 0, sum_x2 = 0, sum_y2 = 0;
        for (const auto& corr : correspondences) {
            sum_x1 += corr.first.x;
            sum_y1 += corr.first.y;
            sum_x2 += corr.second.x;
            sum_y2 += corr.second.y;
        }
        
        size_t n = correspondences.size();
        double mean_x1 = sum_x1 / n;
        double mean_y1 = sum_y1 / n;
        double mean_x2 = sum_x2 / n;
        double mean_y2 = sum_y2 / n;
        
        // 计算旋转角度
        double numerator = 0, denominator = 0;
        for (const auto& corr : correspondences) {
            double x1 = corr.first.x - mean_x1;
            double y1 = corr.first.y - mean_y1;
            double x2 = corr.second.x - mean_x2;
            double y2 = corr.second.y - mean_y2;
            
            numerator += (x1 * y2 - y1 * x2);
            denominator += (x1 * x2 + y1 * y2);
        }
        
        double delta_theta = std::atan2(numerator, denominator);
        double cos_theta = std::cos(delta_theta);
        double sin_theta = std::sin(delta_theta);
        
        // 计算平移
        double delta_x = mean_x2 - (mean_x1 * cos_theta - mean_y1 * sin_theta);
        double delta_y = mean_y2 - (mean_x1 * sin_theta + mean_y1 * cos_theta);
        
        // 更新位姿
        Pose2D delta_pose(delta_x, delta_y, delta_theta);
        Pose2D new_pose = utils::relativePose(current_pose, delta_pose);
        
        // 检查收敛
        double pose_change = std::sqrt(delta_x*delta_x + delta_y*delta_y + delta_theta*delta_theta);
        if (pose_change < convergence_threshold_) {
            break;
        }
        
        current_pose = new_pose;
    }
    
    return current_pose;
}

FeatureBasedFrontEnd::FeatureBasedFrontEnd() : feature_extraction_threshold_(0.5) {}

bool FeatureBasedFrontEnd::processLaserScan(const LaserScan& scan, const Pose2D& predicted_pose, 
                                          Pose2D& corrected_pose, std::vector<Observation>& observations) {
    // 如果是第一帧，直接使用预测位姿
    if (last_features_.empty()) {
        last_features_ = utils::LaserScanProcessor::extractFeatures(
            utils::LaserScanProcessor::polarToCartesianScan(scan));
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
    
    // 执行基于特征的扫描匹配
    corrected_pose = scanMatchPose(scan, LaserScan{}, predicted_pose);
    
    // 更新内部状态
    last_features_ = utils::LaserScanProcessor::extractFeatures(
        utils::LaserScanProcessor::polarToCartesianScan(scan));
    last_pose_ = corrected_pose;
    
    // 从扫描数据中提取观测
    std::vector<Point2D> global_points = utils::LaserScanProcessor::scanToGlobal(scan, corrected_pose);
    for (size_t i = 0; i < global_points.size(); ++i) {
        observations.emplace_back(global_points[i], scan.ranges[i], scan.angles[i], static_cast<int>(i));
    }
    
    // 调用回调函数
    if (pose_callback_) {
        pose_callback_(corrected_pose);
    }
    
    return true;
}

Pose2D FeatureBasedFrontEnd::scanMatchPose(const LaserScan& current_scan, 
                                          const LaserScan& prev_scan, 
                                          const Pose2D& initial_pose) {
    // 提取当前扫描的特征点
    std::vector<Point2D> curr_points = utils::LaserScanProcessor::polarToCartesianScan(current_scan);
    std::vector<Point2D> curr_features = utils::LaserScanProcessor::extractFeatures(curr_points);
    
    // 在上次的特征中寻找对应点
    std::vector<std::pair<Point2D, Point2D>> correspondences;
    for (const auto& curr_feat : curr_features) {
        double min_dist = std::numeric_limits<double>::max();
        Point2D closest_last_feat;
        bool found = false;
        
        for (const auto& last_feat : last_features_) {
            // 将上一时刻的特征转换到当前坐标系
            Point2D transformed_feat = utils::transformPoint(initial_pose, last_feat);
            double dist = utils::distance(curr_feat, transformed_feat);
            
            if (dist < min_dist && dist < 0.5) { // 距离阈值
                min_dist = dist;
                closest_last_feat = last_feat;
                found = true;
            }
        }
        
        if (found) {
            correspondences.emplace_back(curr_feat, closest_last_feat);
        }
    }
    
    if (correspondences.size() < 2) {
        return initial_pose; // 至少需要2个对应点才能计算变换
    }
    
    // 计算变换
    double sum_x1 = 0, sum_y1 = 0, sum_x2 = 0, sum_y2 = 0;
    for (const auto& corr : correspondences) {
        sum_x1 += corr.first.x;
        sum_y1 += corr.first.y;
        sum_x2 += corr.second.x;
        sum_y2 += corr.second.y;
    }
    
    size_t n = correspondences.size();
    double mean_x1 = sum_x1 / n;
    double mean_y1 = sum_y1 / n;
    double mean_x2 = sum_x2 / n;
    double mean_y2 = sum_y2 / n;
    
    // 计算旋转角度
    double numerator = 0, denominator = 0;
    for (const auto& corr : correspondences) {
        double x1 = corr.first.x - mean_x1;
        double y1 = corr.first.y - mean_y1;
        double x2 = corr.second.x - mean_x2;
        double y2 = corr.second.y - mean_y2;
        
        numerator += (x1 * y2 - y1 * x2);
        denominator += (x1 * x2 + y1 * y2);
    }
    
    double delta_theta = std::atan2(numerator, denominator);
    double cos_theta = std::cos(delta_theta);
    double sin_theta = std::sin(delta_theta);
    
    // 计算平移
    double delta_x = mean_x2 - (mean_x1 * cos_theta - mean_y1 * sin_theta);
    double delta_y = mean_y2 - (mean_x1 * sin_theta + mean_y1 * cos_theta);
    
    // 更新位姿
    Pose2D delta_pose(delta_x, delta_y, delta_theta);
    Pose2D new_pose = utils::relativePose(initial_pose, delta_pose);
    
    return new_pose;
}

} // namespace slam2d