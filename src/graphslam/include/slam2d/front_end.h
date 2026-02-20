#ifndef FRONT_END_H
#define FRONT_END_H

#include <vector>
#include <memory>
#include <functional>
#include "types.h"
#include "map.h"

namespace slam2d {

// 里程计模型参数
struct OdometryParams {
    double alpha1 = 0.01;  // 旋转误差与旋转的关系
    double alpha2 = 0.01;  // 旋转误差与平移的关系
    double alpha3 = 0.01;  // 平移误差与平移的关系
    double alpha4 = 0.01;  // 平移误差与旋转的关系
    double alpha5 = 0.01;  // 角度测量噪声
    
    OdometryParams() = default;
    OdometryParams(double a1, double a2, double a3, double a4, double a5)
        : alpha1(a1), alpha2(a2), alpha3(a3), alpha4(a4), alpha5(a5) {}
};

// 前端处理类
class FrontEnd {
public:
    FrontEnd();
    virtual ~FrontEnd() = default;

    // 处理激光雷达数据
    virtual bool processLaserScan(const LaserScan& scan, const Pose2D& predicted_pose, 
                                 Pose2D& corrected_pose, std::vector<Observation>& observations) = 0;

    // 设置里程计参数
    void setOdometryParams(const OdometryParams& params) { odometry_params_ = params; }

    // 设置地图指针
    void setMap(MapPtr map) { map_ = map; }

    // 设置回调函数
    using PoseUpdateCallback = std::function<void(const Pose2D&)>;
    void setPoseUpdateCallback(PoseUpdateCallback callback) { pose_callback_ = callback; }

protected:
    OdometryParams odometry_params_;
    MapPtr map_;
    PoseUpdateCallback pose_callback_;

    // 使用运动模型预测位姿
    Pose2D predictPose(const Pose2D& prev_pose, const Pose2D& control);

    // 基于扫描匹配校正位姿
    virtual Pose2D scanMatchPose(const LaserScan& current_scan, 
                                const LaserScan& prev_scan, 
                                const Pose2D& initial_pose) = 0;
};

// ICP前端实现
class ICPFrontEnd : public FrontEnd {
public:
    ICPFrontEnd();
    virtual ~ICPFrontEnd() = default;

    // 处理激光雷达数据
    bool processLaserScan(const LaserScan& scan, const Pose2D& predicted_pose, 
                         Pose2D& corrected_pose, std::vector<Observation>& observations) override;

protected:
    // ICP扫描匹配
    Pose2D scanMatchPose(const LaserScan& current_scan, 
                        const LaserScan& prev_scan, 
                        const Pose2D& initial_pose) override;

private:
    LaserScan last_scan_;
    Pose2D last_pose_;
    int max_iterations_;
    double convergence_threshold_;
};

// 特征前端实现
class FeatureBasedFrontEnd : public FrontEnd {
public:
    FeatureBasedFrontEnd();
    virtual ~FeatureBasedFrontEnd() = default;

    // 处理激光雷达数据
    bool processLaserScan(const LaserScan& scan, const Pose2D& predicted_pose, 
                         Pose2D& corrected_pose, std::vector<Observation>& observations) override;

protected:
    // 基于特征的扫描匹配
    Pose2D scanMatchPose(const LaserScan& current_scan, 
                        const LaserScan& prev_scan, 
                        const Pose2D& initial_pose) override;

private:
    std::vector<Point2D> last_features_;
    Pose2D last_pose_;
    double feature_extraction_threshold_;
};

using FrontEndPtr = std::shared_ptr<FrontEnd>;

} // namespace slam2d

#endif // FRONT_END_H