# 项目使用说明

## 可执行脚本 (bin/)
- `./bin/graphslam` - 启动GraphSLAM程序
- `./bin/graphslam-complete` - 完整环境的GraphSLAM启动
- `./bin/rviz` - 启动Rviz可视化
- `./bin/simulation` - 仿真环境启动器

## 工具脚本 (tools/)
- `./tools/recovery` - NoMachine故障恢复

## ROS 2包 (src/)
标准ROS 2包结构，使用colcon构建

## 使用示例:
```bash
cd /home/chuil/Desktop/zhongxi
./bin/rviz                    # 启动可视化
./bin/graphslam              # 启动SLAM
./tools/recovery             # 系统恢复
```
