# GraphSLAM 编译问题解决方案

## 问题描述
在 Jetson Orin Nano 上编译 GraphSLAM 包时遇到以下问题：
1. `undefined reference to 'main'` 链接错误
2. 包名不符合ROS2规范警告

## 解决步骤

### 1. 诊断问题
```bash
# 在Jetson上执行诊断脚本
cd ~/Desktop/zhongxi/zhongxi_2_5/src/GraphSLAM
chmod +x diagnose_issue.sh
./diagnose_issue.sh
```

### 2. 执行修复
```bash
# 运行自动修复脚本
chmod +x fix_compilation.sh
./fix_compilation.sh
```

### 3. 手动检查要点

#### 检查主函数文件
```bash
# 确认node目录中包含main函数
grep -rn "int main" ~/Desktop/zhongxi/zhongxi_2_5/src/GraphSLAM/src/node/
```

#### 检查CMakeLists.txt配置
确认以下内容正确：
- `NODE_SOURCES` 变量正确包含了主函数文件
- `file(GLOB_RECURSE NODE_SOURCES "src/node/*.cpp")` 能匹配到实际文件

#### 包名规范化（推荐长期方案）
将包名从 `GraphSLAM` 改为 `graph_slam`：
1. 重命名目录：`mv GraphSLAM graph_slam`
2. 修改 `package.xml` 中的 `<name>` 标签
3. 更新所有相关的CMakeLists.txt引用

### 4. 依赖检查
确保安装了必要依赖：
```bash
sudo apt update
sudo apt install libqglviewer-dev-qt5 libg2o-dev libeigen3-dev
```

### 5. 编译命令
```bash
# 清理后重新编译
cd ~/Desktop/zhongxi/zhongxi_2_5
rm -rf build/GraphSLAM install/GraphSLAM log/GraphSLAM
colcon build --packages-select GraphSLAM
```

## 常见问题排查

1. **找不到main函数**：
   - 确认 `src/node/` 目录中确实包含带有 `int main()` 的cpp文件
   - 检查文件是否被正确包含在 `NODE_SOURCES` 中

2. **链接错误**：
   - 确保所有源文件都被正确编译
   - 检查是否有重复定义或缺失的符号

3. **包名警告**：
   - 这只是警告，不影响编译
   - 但建议按ROS2规范修改包名为小写格式

## 验证编译结果
```bash
# 检查生成的可执行文件
find ~/Desktop/zhongxi/zhongxi_2_5/install -name "*graphslam*" -type f

# 测试运行
source ~/Desktop/zhongxi/zhongxi_2_5/install/setup.bash
ros2 run GraphSLAM graphslam_node --help
```