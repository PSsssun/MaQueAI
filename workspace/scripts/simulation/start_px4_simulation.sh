#!/bin/bash

echo "🚁 === MaQueAI PX4仿真环境启动脚本 ==="
echo "Author: MaQueAI Team"
echo "Description: 在宿主机启动PX4 SITL + Gazebo Classic仿真"
echo ""

# 检查PX4路径
PX4_PATH="${1:-/home/$(whoami)/PX4-Autopilot}"

if [ ! -d "$PX4_PATH" ]; then
    echo "❌ PX4路径不存在: $PX4_PATH"
    echo "💡 请提供正确的PX4路径:"
    echo "   $0 /path/to/PX4-Autopilot"
    echo ""
    echo "🔧 如果没有PX4，可以这样安装:"
    echo "   cd ~ && git clone --recursive https://github.com/PX4/PX4-Autopilot.git"
    echo "   cd PX4-Autopilot && bash ./Tools/setup/ubuntu.sh"
    exit 1
fi

echo "📍 使用PX4路径: $PX4_PATH"
cd "$PX4_PATH"

# 检查必要文件
if [ ! -f "Tools/sitl_gazebo/worlds/iris_lidar.world" ] && [ ! -f "Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/iris_lidar.world" ]; then
    echo "⚠️  LiDAR仿真世界文件不存在，使用默认世界"
    WORLD_FILE="empty"
else
    echo "✅ 找到LiDAR仿真世界文件"
    WORLD_FILE="iris_lidar"
fi

# 设置环境变量
export PX4_HOME_LAT=39.9612      # 北京纬度
export PX4_HOME_LON=116.3348     # 北京经度
export PX4_HOME_ALT=39           # 海拔高度
export PX4_SIM_MODEL=iris        # 无人机模型

echo ""
echo "🌍 === 仿真环境配置 ==="
echo "📍 起始位置: 纬度=$PX4_HOME_LAT, 经度=$PX4_HOME_LON, 高度=$PX4_HOME_ALT"
echo "🚁 无人机模型: $PX4_SIM_MODEL"
echo "🗺️  仿真世界: $WORLD_FILE"
echo ""

# 检查Gazebo版本
if command -v gazebo &> /dev/null; then
    GAZEBO_VERSION=$(gazebo --version | head -1)
    echo "🎮 Gazebo版本: $GAZEBO_VERSION"
else
    echo "⚠️  Gazebo未安装，尝试使用系统默认"
fi

echo ""
echo "🚀 === 启动PX4仿真 ==="
echo "⏳ 正在启动PX4 SITL + Gazebo..."
echo ""

# 清理之前的构建
make clean > /dev/null 2>&1

# 启动仿真
echo "🎯 执行命令: make px4_sitl gazebo_$PX4_SIM_MODEL"

if make px4_sitl gazebo_$PX4_SIM_MODEL; then
    echo ""
    echo "✅ PX4仿真启动成功！"
    echo ""
    echo "📋 === 仿真信息 ==="
    echo "🔗 MAVLink端口: 14540 (UDP)"
    echo "🔗 仿真连接端口: 14560 (TCP)"
    echo "🖥️  QGroundControl连接: 自动连接到 UDP 14540"
    echo ""
    echo "💡 === Docker容器连接 ==="
    echo "1. 启动Docker容器:"
    echo "   cd MaQueAI && ./scripts/run_docker_dev.sh"
    echo ""
    echo "2. 在容器内启动ROS系统:"
    echo "   build_all.sh"
    echo "   source /workspace/catkin_ws/devel/setup.bash"
    echo "   roslaunch maque_system maque_system.launch"
    echo ""
    echo "🎮 === 控制说明 ==="
    echo "- 使用QGroundControl进行手动控制"
    echo "- 或使用ROS自主控制器"
    echo "- Ctrl+C 退出仿真"
else
    echo ""
    echo "❌ PX4仿真启动失败"
    echo ""
    echo "🔧 === 可能的解决方案 ==="
    echo "1. 检查依赖是否完整安装:"
    echo "   cd $PX4_PATH && bash ./Tools/setup/ubuntu.sh"
    echo ""
    echo "2. 手动构建PX4:"
    echo "   cd $PX4_PATH && make px4_sitl"
    echo ""
    echo "3. 检查Gazebo是否正确安装:"
    echo "   gazebo --version"
    echo ""
    exit 1
fi 