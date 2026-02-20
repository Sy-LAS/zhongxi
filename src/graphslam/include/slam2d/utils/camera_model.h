#ifndef CAMERA_MODEL_H
#define CAMERA_MODEL_H

#include <vector>
#include <Eigen/Core>
#include "slam2d/types.h"

namespace slam2d {
namespace utils {

// 相机内参
struct CameraIntrinsics {
    double fx = 0.0;   // x轴焦距（像素）
    double fy = 0.0;   // y轴焦距（像素）
    double cx = 0.0;   // x轴主点（像素）
    double cy = 0.0;   // y轴主点（像素）
    
    CameraIntrinsics() = default;
    CameraIntrinsics(double f_x, double f_y, double c_x, double c_y)
        : fx(f_x), fy(f_y), cx(c_x), cy(c_y) {}
};

// 相机畸变参数
struct DistortionCoefficients {
    double k1 = 0.0;   // 径向畸变系数1
    double k2 = 0.0;   // 径向畸变系数2
    double p1 = 0.0;   // 切向畸变系数1
    double p2 = 0.0;   // 切向畸变系数2
    double k3 = 0.0;   // 径向畸变系数3
    
    DistortionCoefficients() = default;
    DistortionCoefficients(double k_1, double k_2, double p_1, double p_2, double k_3)
        : k1(k_1), k2(k_2), p1(p_1), p2(p_2), k3(k_3) {}
};

// 相机模型类
class CameraModel {
public:
    CameraModel() = default;
    CameraModel(const CameraIntrinsics& intrinsics, 
               const DistortionCoefficients& distortion = DistortionCoefficients());
    
    // 将3D点投影到2D图像坐标
    Point2D project3DTo2D(const Point2D& point3d) const;
    
    // 将2D图像坐标反投影到3D空间
    Point2D unproject2DTo3D(const Point2D& point2d) const;
    
    // 畸变校正
    Point2D undistortPoint(const Point2D& distorted_point) const;
    
    // 设置相机内参
    void setIntrinsics(const CameraIntrinsics& intrinsics) { intrinsics_ = intrinsics; }
    
    // 设置畸变参数
    void setDistortionCoefficients(const DistortionCoefficients& coeffs) { distortion_coeffs_ = coeffs; }
    
    // 获取相机内参
    const CameraIntrinsics& getIntrinsics() const { return intrinsics_; }
    
    // 获取畸变参数
    const DistortionCoefficients& getDistortionCoefficients() const { return distortion_coeffs_; }
    
    // 计算重投影误差
    static double computeReprojectionError(const std::vector<Point2D>& observed_points,
                                         const std::vector<Point2D>& projected_points);

private:
    CameraIntrinsics intrinsics_;
    DistortionCoefficients distortion_coeffs_;
    
    // 应用径向和切向畸变
    Point2D applyDistortion(const Point2D& normalized_point) const;
};

// 双目相机模型
class StereoCameraModel {
public:
    StereoCameraModel(const CameraModel& left_camera, 
                     const CameraModel& right_camera,
                     const Pose2D& baseline);  // 基线（右相机相对于左相机的位姿）
    
    // 从视差计算深度
    double disparityToDepth(double disparity) const;
    
    // 三角化计算3D点
    Point2D triangulate(const Point2D& left_point, const Point2D& right_point) const;
    
private:
    CameraModel left_camera_;
    CameraModel right_camera_;
    Pose2D baseline_;
};

} // namespace utils
} // namespace slam2d

#endif // CAMERA_MODEL_H