#include "utils/transformations.h"
#include <cmath>

namespace slam2d {
namespace utils {

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Transform2D poseToTransform(const Pose2D& pose) {
    double cos_theta = std::cos(pose.theta);
    double sin_theta = std::sin(pose.theta);

    Transform2D transform = Transform2D::Identity();
    transform << cos_theta, -sin_theta, pose.x,
                 sin_theta,  cos_theta, pose.y,
                         0,          0,      1;
    return transform;
}

Pose2D transformToPose(const Transform2D& transform) {
    double x = transform(0, 2);
    double y = transform(1, 2);
    double theta = std::atan2(transform(1, 0), transform(0, 0));
    return Pose2D(x, y, theta);
}

Pose2D relativePose(const Pose2D& from, const Pose2D& to) {
    Transform2D from_transform = poseToTransform(from);
    Transform2D to_transform = poseToTransform(to);
    
    Transform2D relative_transform = from_transform.inverse() * to_transform;
    return transformToPose(relative_transform);
}

Point2D transformPoint(const Pose2D& pose, const Point2D& point) {
    Transform2D transform = poseToTransform(pose);
    Eigen::Vector3d point_homogeneous(point.x, point.y, 1.0);
    Eigen::Vector3d transformed_point = transform * point_homogeneous;
    
    return Point2D(transformed_point.x(), transformed_point.y());
}

Point2D localToGlobal(const Pose2D& pose, const Point2D& point) {
    return transformPoint(pose, point);
}

Point2D globalToLocal(const Pose2D& pose, const Point2D& point) {
    Transform2D transform = poseToTransform(pose);
    Transform2D inv_transform = transform.inverse();
    
    Eigen::Vector3d point_homogeneous(point.x, point.y, 1.0);
    Eigen::Vector3d transformed_point = inv_transform * point_homogeneous;
    
    return Point2D(transformed_point.x(), transformed_point.y());
}

double distance(const Point2D& p1, const Point2D& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx*dx + dy*dy);
}

double distanceFromOrigin(const Point2D& p) {
    return std::sqrt(p.x*p.x + p.y*p.y);
}

double bearingAngle(const Pose2D& pose, const Point2D& point) {
    Point2D local_point = globalToLocal(pose, point);
    return std::atan2(local_point.y, local_point.x);
}

double poseDistance(const Pose2D& p1, const Pose2D& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx*dx + dy*dy);
}

Point2D polarToCartesian(double range, double bearing) {
    double x = range * std::cos(bearing);
    double y = range * std::sin(bearing);
    return Point2D(x, y);
}

Eigen::Matrix2d rotationJacobian(double theta) {
    Eigen::Matrix2d J;
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    
    J << -sin_theta, -cos_theta,
          cos_theta, -sin_theta;
    return J;
}

} // namespace utils
} // namespace slam2d