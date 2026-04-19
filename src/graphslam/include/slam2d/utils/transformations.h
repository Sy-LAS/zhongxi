#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include "slam2d/types.h"
#include "slam2d/map.h"
#include <random>

namespace slam2d {
namespace utils {

/**
 * @brief 将角度规范化到 [-π, π] 区间
 * @param angle 输入角度
 * @return 规范化后的角度
 */
double normalizeAngle(double angle);

/**
 * @brief 从位姿创建2D变换矩阵
 * @param pose 位姿 (x, y, theta)
 * @return 3x3变换矩阵
 */
Transform2D poseToTransform(const Pose2D& pose);

/**
 * @brief 从变换矩阵提取位姿
 * @param transform 3x3变换矩阵
 * @return 位姿 (x, y, theta)
 */
Pose2D transformToPose(const Transform2D& transform);

/**
 * @brief 计算两个位姿之间的相对变换
 * @param from 起始位姿
 * @param to 目标位姿
 * @return 相对变换
 */
Pose2D relativePose(const Pose2D& from, const Pose2D& to);

/**
 * @brief 将点从一个坐标系变换到另一个坐标系
 * @param pose 坐标系位姿
 * @param point 待变换的点
 * @return 变换后的点
 */
Point2D transformPoint(const Pose2D& pose, const Point2D& point);

/**
 * @brief 将点从局部坐标系变换到全局坐标系
 * @param pose 局部位姿
 * @param point 局部坐标系中的点
 * @return 全局坐标系中的点
 */
Point2D localToGlobal(const Pose2D& pose, const Point2D& point);

/**
 * @brief 将点从全局坐标系变换到局部坐标系
 * @param pose 局部位姿
 * @param point 全局坐标系中的点
 * @return 局部坐标系中的点
 */
Point2D globalToLocal(const Pose2D& pose, const Point2D& point);

/**
 * @brief 计算两点之间的欧几里得距离
 * @param p1 第一个点
 * @param p2 第二个点
 * @return 两点间的距离
 */
double distance(const Point2D& p1, const Point2D& p2);

/**
 * @brief 计算点到原点的距离
 * @param p 点
 * @return 点到原点的距离
 */
double distanceFromOrigin(const Point2D& p);

/**
 * @brief 计算位姿与点之间的相对角度
 * @param pose 位姿
 * @param point 点
 * @return 相对角度
 */
double bearingAngle(const Pose2D& pose, const Point2D& point);

/**
 * @brief 计算两个位姿之间的距离
 * @param p1 第一个位姿
 * @param p2 第二个位姿
 * @return 位姿间的距离
 */
double poseDistance(const Pose2D& p1, const Pose2D& p2);

/**
 * @brief 将极坐标转换为笛卡尔坐标
 * @param range 距离
 * @param bearing 方位角
 * @return 笛卡尔坐标点
 */
Point2D polarToCartesian(double range, double bearing);

/**
 * @brief 计算旋转矩阵的雅可比矩阵
 * @param theta 旋转角度
 * @return 雅可比矩阵
 */
Eigen::Matrix2d rotationJacobian(double theta);

/**
 * @brief 生成正态分布的样本
 * @param mean 均值
 * @param stddev 标准差
 * @return 正态分布样本
 */
double sampleNormalDistribution(double mean, double stddev);

} // namespace utils
} // namespace slam2d

#endif // TRANSFORMATIONS_H