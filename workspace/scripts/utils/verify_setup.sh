#!/bin/bash

echo "🔍 === MaQueAI 项目结构验证脚本 ==="
echo "Author: MaQueAI Team"
echo "Description: 验证项目结构和组件是否正确设置"
echo ""

cd "$(dirname "$0")/.."

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 验证函数
check_exists() {
    if [ -e "$1" ]; then
        echo -e "${GREEN}✅ $1${NC}"
        return 0
    else
        echo -e "${RED}❌ $1${NC}"
        return 1
    fi
}

check_executable() {
    if [ -x "$1" ]; then
        echo -e "${GREEN}✅ $1 (可执行)${NC}"
        return 0
    else
        echo -e "${YELLOW}⚠️  $1 (不可执行)${NC}"
        return 1
    fi
}

echo "📁 === 验证目录结构 ==="
check_exists "workspace"
check_exists "workspace/src"
check_exists "workspace/src/LIVO"
check_exists "workspace/px4"
check_exists "data"
check_exists "external"
check_exists "docker_config"
check_exists "scripts"
check_exists "docs"

echo ""
echo "📄 === 验证关键文件 ==="
check_exists "workspace/src/LIVO/CMakeLists.txt"
check_exists "workspace/src/LIVO/include/frame.h"
check_exists "workspace/px4/CMakeLists.txt"
check_exists "workspace/px4/Tools/setup/ubuntu.sh"
check_exists "docker_config/Dockerfile-MaQueAI"
check_exists ".gitignore"
check_exists ".clang-format"

echo ""
echo "🔧 === 验证脚本权限 ==="
check_executable "scripts/build_docker.sh"
check_executable "scripts/run_docker.sh"
check_executable "scripts/run_docker_dev.sh"
check_executable "scripts/build_livo.sh"
check_executable "scripts/build_px4.sh"
check_executable "scripts/run_format.sh"

echo ""
echo "📊 === 项目统计信息 ==="
echo "LIVO源码文件数: $(find workspace/src/LIVO -name "*.cpp" -o -name "*.h" -o -name "*.hpp" | wc -l)"
echo "PX4源码文件数: $(find workspace/px4 -name "*.cpp" -o -name "*.c" -o -name "*.h" -o -name "*.hpp" 2>/dev/null | wc -l)"
echo "脚本文件数: $(find scripts -name "*.sh" | wc -l)"
echo "总项目大小: $(du -sh . 2>/dev/null | cut -f1)"

echo ""
echo "🐳 === Docker映射验证 ==="
echo "标准模式映射:"
echo "  workspace/src → /workspace/catkin_ws/src (ROS项目)"
echo "  workspace/px4 → /workspace/px4 (PX4项目)"
echo "  data → /data (数据文件)"

echo ""
echo "开发模式映射:"
echo "  workspace → /workspace (完整工作空间)"
echo "  data → /data (数据文件)"

echo ""
echo "🎯 === 使用建议 ==="
echo "1. 构建镜像: ./scripts/build_docker.sh"
echo "2. 启动开发容器: ./scripts/run_docker_dev.sh"
echo "3. 编译LIVO: build_livo.sh (容器内)"
echo "4. 编译PX4: build_px4.sh (容器内)"

echo ""
echo "📚 === 文档位置 ==="
echo "项目结构说明: docs/PROJECT_STRUCTURE.md"
echo "更多文档请查看: docs/ 目录"

echo ""
if [ $? -eq 0 ]; then
    echo -e "${GREEN}🎉 项目结构验证完成！${NC}"
else
    echo -e "${YELLOW}⚠️  项目结构验证完成，但发现一些问题，请检查上述输出${NC}"
fi 