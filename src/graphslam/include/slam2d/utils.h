#ifndef SLAM_UTILS_H
#define SLAM_UTILS_H

#include <vector>
#include "types.h"

namespace slam2d {

class SLAMUtils {
public:
    /**
     * @brief 计算两个位姿之间的误差
     * @param pose1 第一个位姿
     * @param pose2 第二个位姿
     * @return 位姿误差
     */
    static double poseError(const Pose2D& pose1, const Pose2D& pose2);
    static double sampleNormalDistribution(double mean = 0.0, double stddev = 1.0);
    /**
     * @brief 计算雅可比矩阵
     * @param from 起始位姿
     * @param to 终止位姿
     * @param z 测量值
     * @return 雅可比矩阵
     */
    static Eigen::Matrix3d computeJacobian(const Pose2D& from, const Pose2D& to, const Pose2D& z);

    /**
     * @brief 数据关联 - 寻找最近的路标点
     * @param observed_point 观测到的点
     * @param landmarks 路标点集合
     * @param threshold 关联阈值
     * @return 关联的路标点索引，-1表示未找到匹配
     */
    static int dataAssociation(const Point2D& observed_point, 
                              const std::vector<Point2D>& landmarks, 
                              double threshold = 1.0);

    /**
     * @brief 计算信息矩阵（协方差矩阵的逆）
     * @param covariance 协方差矩阵
     * @return 信息矩阵
     */
    static Covariance2D computeInformationMatrix(const Covariance2D& covariance);

    /**
     * @brief 从激光扫描数据中提取直线特征
     * @param scan 激光扫描数据
     * @return 直线段集合
     */
    static std::vector<std::pair<Point2D, Point2D>> extractLinesFromScan(const LaserScan& scan);

    /**
     * @brief 计算回环检测得分
     * @param current_pose 当前位姿
     * @param candidate_poses 候选位姿
     * @param threshold 阈值
     * @return 是否检测到回环
     */
    static bool loopDetection(const Pose2D& current_pose, 
                            const std::vector<Pose2D>& candidate_poses,
                            double threshold = 2.0);

    /**
     * @brief 计算路径长度
     * @param trajectory 机器人轨迹
     * @return 路径总长度
     */
    static double calculatePathLength(const std::vector<Pose2D>& trajectory);

    /**
     * @brief 平滑轨迹
     * @param trajectory 原始轨迹
     * @param window_size 平滑窗口大小
     * @return 平滑后的轨迹
     */
    static std::vector<Pose2D> smoothTrajectory(const std::vector<Pose2D>& trajectory, 
                                               int window_size = 3);

    /**
     * @brief 评估地图质量
     * @param map 占用栅格地图
     * @param width 地图宽度
     * @param height 地图高度
     * @return 地图质量分数
     */
    static double evaluateMapQuality(const std::vector<GridCell>& map, 
                                   int width, int height);
};

} // namespace slam2d

#endif // SLAM_UTILS_H    static double sampleNormalDistribution(double mean = 0.0, double stddev = 1.0);
