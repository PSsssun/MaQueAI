#!/bin/bash

echo "🖥️ === MaQueAI GUI-enabled Docker Environment ==="
echo "启动支持图形界面的Docker容器，可运行Gazebo可视化仿真"
echo ""

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# 检查是否在正确的项目根目录
if [ ! -d "$PROJECT_ROOT/workspace" ] || [ ! -d "$PROJECT_ROOT/workspace/px4" ]; then
    echo "❌ 错误：无法找到MaQueAI项目结构"
    echo "请确保此脚本在正确的MaQueAI项目目录中运行"
    echo "📂 当前PROJECT_ROOT: $PROJECT_ROOT"
    echo "📂 查找: $PROJECT_ROOT/workspace 和 $PROJECT_ROOT/workspace/px4"
    echo "📂 SCRIPT_DIR: $SCRIPT_DIR"
    ls -la "$PROJECT_ROOT" 2>/dev/null || echo "❌ PROJECT_ROOT目录不存在"
    exit 1
fi

# 检查Docker是否运行
if ! docker info >/dev/null 2>&1; then
    echo "❌ Docker未运行。请启动Docker后重试。"
    exit 1
fi

# Docker镜像和容器名称
IMAGE_NAME="maque-ai:latest"
CONTAINER_NAME="maque-ai-gui-container"

# 检查Docker镜像是否存在
if ! docker images | grep -q "maque-ai"; then
    echo "❌ Docker镜像 $IMAGE_NAME 不存在"
    echo "💡 请先构建镜像或确保镜像名称正确"
    exit 1
fi
echo "✅ Docker镜像 $IMAGE_NAME 已找到"



# GUI环境检测
check_gui_support() {
    echo "🔍 检查GUI支持..."
    
    # 检查X11服务器
    if [ -z "$DISPLAY" ]; then
        echo "⚠️ 警告：未设置DISPLAY环境变量"
        export DISPLAY=:0
        echo "已设置 DISPLAY=$DISPLAY"
    fi
    
    # 检查X11授权
    if [ ! -f "$HOME/.Xauthority" ]; then
        echo "⚠️ 警告：未找到X11授权文件，创建空文件"
        touch "$HOME/.Xauthority"
    fi
    
    # 检查DRI设备（GPU加速）
    GPU_DEVICE_AVAILABLE=false
    if [ -d "/dev/dri" ]; then
        echo "✅ 检测到GPU设备，启用硬件加速"
        GPU_DEVICE_AVAILABLE=true
    else
        echo "ℹ️ 未检测到GPU设备，使用软件渲染"
    fi
}

# 准备Docker运行参数
prepare_docker_args() {
    check_gui_support
    
    # 基础参数
    DOCKER_ARGS=(
        "--name" "$CONTAINER_NAME"
        "--rm"
        "--interactive"
        "--tty"
        "--privileged"
        "--network=host"
        
        # X11支持
        "--env" "DISPLAY=$DISPLAY"
        "--env" "QT_X11_NO_MITSHM=1"
        "--env" "QT_QPA_PLATFORM=xcb"
        "--env" "LIBGL_ALWAYS_INDIRECT=1"
        "--env" "LIBGL_ALWAYS_SOFTWARE=1"
        "--env" "MESA_GL_VERSION_OVERRIDE=3.3"
        "--env" "MESA_GLSL_VERSION_OVERRIDE=330"
        "--volume" "/tmp/.X11-unix:/tmp/.X11-unix:rw"
        "--volume" "$HOME/.Xauthority:/home/ros/.Xauthority:rw"
        
        # 项目目录挂载
        "--volume" "$PROJECT_ROOT/workspace:/workspace"
        "--volume" "$PROJECT_ROOT:/maque-ai-root"
        
        # 工作目录
        "--workdir" "/workspace"
    )
    
    # 添加GPU支持（如果可用）
    if [ "$GPU_DEVICE_AVAILABLE" = true ]; then
        DOCKER_ARGS+=("--device=/dev/dri")
    fi
    
    # 添加声音支持（如果设备存在）
    if [ -e "/dev/snd" ]; then
        DOCKER_ARGS+=("--device" "/dev/snd")
    fi
}

# 停止已存在的容器
cleanup_existing_container() {
    if docker ps -a | grep -q "$CONTAINER_NAME"; then
        echo "🧹 清理已存在的容器..."
        docker stop "$CONTAINER_NAME" >/dev/null 2>&1 || true
        docker rm "$CONTAINER_NAME" >/dev/null 2>&1 || true
    fi
}

# 快速启动仿真
quick_start_simulation() {
    local sim_type="$1"
    echo "🚀 快速启动PX4仿真模式: $sim_type"
    
    # 清理已存在的容器
    cleanup_existing_container
    
    # 准备Docker参数
    prepare_docker_args
    
    # 允许X11连接
    echo "🔐 配置X11权限..."
    xhost +local:docker >/dev/null 2>&1 || echo "⚠️ 无法设置X11权限，GUI可能无法正常工作"
    
    echo "🐳 启动Docker容器并运行仿真..."
    echo "📂 项目目录: $PROJECT_ROOT"
    echo "🖥️ DISPLAY: $DISPLAY"
    echo "🎮 仿真类型: $sim_type"
    echo ""
    
    # 根据仿真类型选择启动命令
    local sim_cmd
    case "$sim_type" in
        "iris"|"default")
            sim_cmd="cd /workspace/px4 && make px4_sitl gazebo_iris"
            ;;
        "plane")
            sim_cmd="cd /workspace/px4 && make px4_sitl gazebo_plane"
            ;;
        "vtol")
            sim_cmd="cd /workspace/px4 && make px4_sitl gazebo_vtol_standard"
            ;;
        "rover")
            sim_cmd="cd /workspace/px4 && make px4_sitl gazebo_rover"
            ;;
        *)
            sim_cmd="cd /workspace/px4 && make px4_sitl gazebo"
            ;;
    esac
    
    # 启动容器并运行仿真
    docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME" bash -c "$sim_cmd; bash" || {
        echo "❌ Docker容器启动失败"
        echo ""
        echo "💡 故障排除建议："
        echo "   1. 确保Docker镜像存在: docker images | grep maque-ai"
        echo "   2. 检查X11服务器是否运行"
        echo "   3. 尝试运行: xhost +local:docker"
        echo "   4. 如果在WSL中，确保X11服务器已启动"
        echo "   5. 确保PX4已编译: ls /workspace/px4/build/px4_sitl_default/bin/px4"
        exit 1
    }
    
    # 恢复X11权限
    echo "🔒 恢复X11权限..."
    xhost -local:docker >/dev/null 2>&1 || true
}

# 主函数
main() {
    echo "🚀 启动GUI支持的MaQueAI开发环境..."
    
    # 清理已存在的容器
    cleanup_existing_container
    
    # 准备Docker参数
    prepare_docker_args
    
    # 允许X11连接
    echo "🔐 配置X11权限..."
    xhost +local:docker >/dev/null 2>&1 || echo "⚠️ 无法设置X11权限，GUI可能无法正常工作"
    
    echo "🐳 启动Docker容器..."
    echo "📂 项目目录: $PROJECT_ROOT"
    echo "🖥️ DISPLAY: $DISPLAY"
    echo ""
    
    # 启动容器并显示使用说明
    docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME" bash -c "
        echo '🎉 欢迎使用MaQueAI GUI开发环境!'
        echo ''
        echo '📋 快速启动仿真:'
        echo '   cd /workspace/px4'
        echo '   make px4_sitl gazebo           # 默认iris四旋翼仿真'
        echo '   make px4_sitl gazebo_plane     # 固定翼仿真'
        echo '   make px4_sitl gazebo_vtol_standard # 垂直起降仿真'
        echo ''
        echo '🔍 检查PX4编译状态:'
        if [ -f /workspace/px4/build/px4_sitl_default/bin/px4 ]; then
            echo '   ✅ PX4已编译完成，可以直接启动仿真'
        else
            echo '   ❌ PX4未编译，请先运行构建脚本:'
            echo '      ./scripts/build/build_px4.sh'
        fi
        echo ''
        echo '🌍 设置Gazebo环境变量...'
        export GAZEBO_MODEL_PATH=\"/workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:\$GAZEBO_MODEL_PATH\"
        export GAZEBO_PLUGIN_PATH=\"/workspace/px4/build/px4_sitl_default/build_gazebo-classic:\$GAZEBO_PLUGIN_PATH\"
        export GAZEBO_RESOURCE_PATH=\"/workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds:\$GAZEBO_RESOURCE_PATH\"
        echo '   ✅ Gazebo环境变量已设置'
        echo ''
        echo '💡 如果是首次使用，请先运行:'
        echo '   ./scripts/build/build_px4.sh    # 完整构建PX4和Gazebo环境'
        echo ''
        echo '📖 输入 exit 退出容器'
        echo '🆘 输入 --help 查看完整使用说明'
        echo ''
        cd /workspace
        bash
    " || {
        echo "❌ Docker容器启动失败"
        echo ""
        echo "💡 故障排除建议："
        echo "   1. 确保Docker镜像存在: docker images | grep maque-ai"
        echo "   2. 检查X11服务器是否运行"
        echo "   3. 尝试运行: xhost +local:docker"
        echo "   4. 如果在WSL中，确保X11服务器已启动"
        echo "   5. 运行构建脚本: ./scripts/build/build_px4.sh"
        exit 1
    }
    
    # 恢复X11权限
    echo "🔒 恢复X11权限..."
    xhost -local:docker >/dev/null 2>&1 || true
}

# 显示使用说明
show_usage() {
    echo "📖 GUI容器使用说明："
    echo ""
    echo "🖥️ 启动PX4 Gazebo仿真："
    echo "   cd /workspace/px4"
    echo "   make px4_sitl gazebo                    # 默认iris四旋翼"
    echo "   make px4_sitl gazebo_iris               # iris四旋翼模型"
    echo "   make px4_sitl gazebo_plane              # 固定翼飞机"
    echo "   make px4_sitl gazebo_vtol_standard      # 垂直起降飞机"
    echo "   make px4_sitl gazebo_rover              # 地面车辆"
    echo ""
    echo "🌍 使用不同世界环境："
    echo "   make px4_sitl gazebo_iris__empty        # 空白世界"
    echo "   make px4_sitl gazebo_iris__warehouse     # 仓库世界"
    echo "   make px4_sitl gazebo_iris__windy        # 有风环境"
    echo ""
    echo "🎮 启动QGroundControl兼容模式："
    echo "   cd /workspace && ./scripts/simulation/start_px4_sim.sh --gui"
    echo ""
    echo "🔧 编译项目（包含完整Gazebo环境配置）："
    echo "   ./scripts/build/build_px4.sh           # 构建PX4和Gazebo环境"
    echo "   ./scripts/build/build_livo.sh          # 构建LIVO系统"
    echo ""
    echo "📋 查看所有可用目标："
    echo "   make list_config_targets                # 查看所有配置目标"
    echo "   make px4_sitl list_vmd_make_targets     # 查看Gazebo模型"
    echo ""
    echo "⚙️ 环境变量已设置："
    echo "   - DISPLAY=$DISPLAY"
    echo "   - 支持OpenGL硬件加速"
    echo "   - 支持音频输出"
    echo "   - MAVLink端口: UDP 14540, TCP 4560"
    echo ""
}

# 检查参数
if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
    show_usage
    exit 0
fi

# 执行主函数
main

echo ""
echo "🎉 GUI容器已退出"
echo "💡 重新启动: $0" 