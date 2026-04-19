#!/bin/bash

echo "优化NoMachine图形性能设置..."

# 创建临时配置文件来降低图形质量
# 这些设置可以通过修改NoMachine配置文件实现

echo "1. 降低图形质量以提升性能..."
echo "建议在NoMachine客户端设置中调整以下选项："
echo "  - Quality: 选择 'Speed' 而不是 'Quality'"
echo "  - Advanced settings -> Graphics: 设置为 'Low color depth'"
echo "  - Disable hardware acceleration if available"
echo "  - Reduce window resolution temporarily"

# 设置环境变量来限制图形性能，减少资源使用
export NX_GLIMITS_PIXMAP_BYTE_SIZE=1000000
export NX_GLIMITS_WINDOW_BYTE_SIZE=1000000
export NX_GLIMITS_PIXMAP_TOTAL_BYTE_SIZE=10000000
export NX_GLIMITS_WINDOW_TOTAL_BYTE_SIZE=10000000

# 限制OpenGL使用
export LIBGL_ALWAYS_SOFTWARE=1

echo "2. 当前设置的环境变量已应用："
echo "   NX_GLIMITS_PIXMAP_BYTE_SIZE=${NX_GLIMITS_PIXMAP_BYTE_SIZE}"
echo "   NX_GLIMITS_WINDOW_BYTE_SIZE=${NX_GLIMITS_WINDOW_BYTE_SIZE}"
echo "   LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}"

echo "3. 建议在运行图形密集应用前执行："
echo "   export LIBGL_ALWAYS_SOFTWARE=1"
echo "   export QT_QUICK_BACKEND=software"
echo "   export MESA_GL_VERSION_OVERRIDE=3.3"

echo "4. 优化完成，这些设置将有助于减少图形卡顿。"
