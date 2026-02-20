#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>

namespace slam2d {
    
// 二维平面的位姿，包含x, y坐标和角度theta
struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    Pose2D() = default;
    Pose2D(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}

    // 从Eigen::Vector3d构造
    explicit Pose2D(const Eigen::Vector3d& vec) : x(vec.x()), y(vec.y()), theta(vec.z()) {}

    // 转换为Eigen::Vector3d
    Eigen::Vector3d toEigen() const {
        return Eigen::Vector3d(x, y, theta);
    }
};

// 二维平面的点，用于地图特征点
struct Point2D {
    double x = 0.0;
    double y = 0.0;

    Point2D() = default;
    Point2D(double x_, double y_) : x(x_), y(y_) {}

    // 从Eigen::Vector2d构造
    explicit Point2D(const Eigen::Vector2d& vec) : x(vec.x()), y(vec.y()) {}

    // 转换为Eigen::Vector2d
    Eigen::Vector2d toEigen() const {
        return Eigen::Vector2d(x, y);
    }
};

// 2D变换矩阵
using Transform2D = Eigen::Matrix3d;

// 协方差矩阵
using Covariance2D = Eigen::Matrix3d;

// 传感器观测数据
struct Observation {
    Point2D point;                    // 观测到的点
    double range = 0.0;              // 距离
    double bearing = 0.0;            // 方位角
    int landmark_id = -1;            // 路标ID
    
    Observation() = default;
    Observation(const Point2D& p, double r, double b, int id) 
        : point(p), range(r), bearing(b), landmark_id(id) {}
};

// 位姿约束，表示两个位姿之间的相对变换
struct Constraint2D {
    int from_id;                     // 起始位姿ID
    int to_id;                       // 终止位姿ID
    Pose2D relative_pose;            // 相对位姿
    Covariance2D information;        // 信息矩阵（协方差的逆）

    Constraint2D() : information(Covariance2D::Identity()) {}
    Constraint2D(int from, int to, const Pose2D& rel_pose, const Covariance2D& info)
        : from_id(from), to_id(to), relative_pose(rel_pose), information(info) {}
};

// 激光雷达扫描数据
struct LaserScan {
    std::vector<double> ranges;      // 距离数据
    std::vector<double> angles;      // 对应的角度
    double min_angle = 0.0;          // 最小角度
    double max_angle = 0.0;          // 最大角度
    double angle_increment = 0.0;    // 角度增量
    double time_increment = 0.0;     // 时间增量
    double scan_time = 0.0;          // 扫描时间
    double range_min = 0.0;          // 最小距离
    double range_max = 0.0;          // 最大距离
    Pose2D origin;                   // 扫描原点位姿

    LaserScan() = default;
    LaserScan(const std::vector<double>& r, const std::vector<double>& a)
        : ranges(r), angles(a) {}
};

// 图优化顶点类型
struct Vertex {
    int id;
    Pose2D pose;
    
    Vertex() = default;
    Vertex(int vertex_id, const Pose2D& p) : id(vertex_id), pose(p) {}
};

// 边（约束）类型
struct Edge {
    int id;
    int vertex_ids[2];               // 连接的两个顶点ID
    Pose2D measurement;              // 测量值
    Covariance2D information_matrix; // 信息矩阵
    
    Edge() : information_matrix(Covariance2D::Identity()) {}
    Edge(int edge_id, int v1, int v2, const Pose2D& meas, const Covariance2D& info)
        : id(edge_id) {
            vertex_ids[0] = v1;
            vertex_ids[1] = v2;
            measurement = meas;
            information_matrix = info;
        }
};

} // namespace slam2d

#endif // TYPES_H