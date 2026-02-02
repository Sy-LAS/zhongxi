#ifndef LASER_SCAN_H
#define LASER_SCAN_H

#include <vector>
#include <Eigen/Core>
#include "slam2d/types.h"

namespace slam2d {
namespace utils {

/**
 * @brief 激光雷达扫描数据处理类
 */
class LaserScanProcessor {
public:
    /**
     * @brief 对激光扫描数据进行预处理，过滤无效距离值
     * @param scan 输入的激光扫描数据
     * @return 过滤后的有效点集合
     */
    static std::vector<Point2D> filterScan(const LaserScan& scan);

    /**
     * @brief 将激光扫描数据从极坐标转换为直角坐标
     * @param scan 激光扫描数据
     * @return 直角坐标系下的点云数据
     */
    static std::vector<Point2D> polarToCartesianScan(const LaserScan& scan);

    /**
     * @brief 将激光扫描数据转换到全局坐标系
     * @param scan 激光扫描数据
     * @param robot_pose 机器人位姿
     * @return 全局坐标系下的点云数据
     */
    static std::vector<Point2D> scanToGlobal(const LaserScan& scan, const Pose2D& robot_pose);

    /**
     * @brief 提取激光扫描数据中的特征点
     * @param points 点云数据
     * @return 特征点集合
     */
    static std::vector<Point2D> extractFeatures(const std::vector<Point2D>& points);

    /**
     * @brief 计算两帧激光扫描数据之间的相似性
     * @param scan1 第一帧扫描数据
     * @param scan2 第二帧扫描数据
     * @return 相似性度量值
     */
    static double scanMatching(const std::vector<Point2D>& scan1, 
                              const std::vector<Point2D>& scan2);

    /**
     * @brief 计算激光雷达数据的运动补偿
     * @param scan 原始激光扫描数据
     * @param start_pose 扫描开始时的机器人位姿
     * @param end_pose 扫描结束时的机器人位姿
     * @return 运动补偿后的扫描数据
     */
    static LaserScan motionCompensation(const LaserScan& scan, 
                                       const Pose2D& start_pose, 
                                       const Pose2D& end_pose);
};

} // namespace utils
} // namespace slam2d

#endif // LASER_SCAN_H