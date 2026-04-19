#include "slam2d/utils/laser_scan.h"
#include "slam2d/utils/transformations.h"
#include <algorithm>
#include <cmath>

namespace slam2d {
namespace utils {

std::vector<Point2D> LaserScanProcessor::filterScan(const LaserScan& scan) {
    std::vector<Point2D> filtered_points;
    filtered_points.reserve(scan.ranges.size());

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        double range = scan.ranges[i];
        // 过滤掉无效距离值
        if (range > scan.range_min && range < scan.range_max) {
            Point2D point = polarToCartesian(range, scan.angles[i]);
            filtered_points.push_back(point);
        }
    }

    return filtered_points;
}

std::vector<Point2D> LaserScanProcessor::polarToCartesianScan(const LaserScan& scan) {
    std::vector<Point2D> points;
    points.reserve(scan.ranges.size());

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        double range = scan.ranges[i];
        if (range > scan.range_min && range < scan.range_max) {
            Point2D point = polarToCartesian(range, scan.angles[i]);
            points.push_back(point);
        }
    }

    return points;
}

std::vector<Point2D> LaserScanProcessor::scanToGlobal(const LaserScan& scan, 
                                                     const Pose2D& robot_pose) {
    std::vector<Point2D> local_points = polarToCartesianScan(scan);
    std::vector<Point2D> global_points;
    global_points.reserve(local_points.size());

    for (const auto& point : local_points) {
        Point2D global_point = localToGlobal(robot_pose, point);
        global_points.push_back(global_point);
    }

    return global_points;
}

std::vector<Point2D> LaserScanProcessor::extractFeatures(const std::vector<Point2D>& points) {
    std::vector<Point2D> features;
    
    // 简单的特征提取方法：提取距离较远的点（可能是角落或边缘）
    for (const auto& point : points) {
        double dist = distanceFromOrigin(point);
        // 如果点距离原点超过一定阈值，则认为是特征点
        if (dist > 1.0) {  // 可调整的阈值
            features.push_back(point);
        }
    }

    return features;
}

double LaserScanProcessor::scanMatching(const std::vector<Point2D>& scan1, 
                                       const std::vector<Point2D>& scan2) {
    if (scan1.empty() || scan2.empty()) {
        return 0.0;
    }

    // 简单的最近邻匹配算法
    double total_distance = 0.0;
    int matched_count = 0;

    for (const auto& point1 : scan1) {
        double min_distance = std::numeric_limits<double>::max();
        
        for (const auto& point2 : scan2) {
            double dist = distance(point1, point2);
            if (dist < min_distance) {
                min_distance = dist;
            }
        }

        if (min_distance < 0.5) {  // 匹配距离阈值
            total_distance += min_distance;
            matched_count++;
        }
    }

    if (matched_count == 0) {
        return 0.0;
    }

    return total_distance / matched_count;
}

LaserScan LaserScanProcessor::motionCompensation(const LaserScan& scan, 
                                                const Pose2D& start_pose, 
                                                const Pose2D& end_pose) {
    LaserScan compensated_scan = scan;
    std::vector<Point2D> points = polarToCartesianScan(scan);

    // 计算扫描过程中机器人的平均运动
    Pose2D avg_motion = relativePose(start_pose, end_pose);
    Pose2D half_motion;
    half_motion.x = avg_motion.x / 2.0;
    half_motion.y = avg_motion.y / 2.0;
    half_motion.theta = avg_motion.theta / 2.0;

    // 对每个点应用运动补偿
    for (size_t i = 0; i < points.size(); ++i) {
        // 根据扫描顺序，对不同角度的点应用不同程度的补偿
        double ratio = static_cast<double>(i) / points.size();
        Pose2D compensation_pose;
        compensation_pose.x = half_motion.x * ratio;
        compensation_pose.y = half_motion.y * ratio;
        compensation_pose.theta = half_motion.theta * ratio;

        Point2D compensated_point = localToGlobal(compensation_pose, points[i]);
        points[i] = compensated_point;
    }

    // 重新计算距离和角度
    compensated_scan.ranges.clear();
    compensated_scan.angles.clear();

    for (const auto& point : points) {
        double range = distanceFromOrigin(point);
        double angle = std::atan2(point.y, point.x);
        compensated_scan.ranges.push_back(range);
        compensated_scan.angles.push_back(angle);
    }

    return compensated_scan;
}

} // namespace utils
} // namespace slam2d