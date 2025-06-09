#!/bin/bash

echo "🚁 === PX4 SITL仿真启动器 ==="
echo "Author: MaQueAI Team"
echo "Description: 智能启动PX4 SITL仿真，支持GUI和headless模式"
echo ""

# 默认参数
VEHICLE="iris"
WORLD="empty"
GUI_MODE="auto"
HEADLESS=false
VERBOSE=false
PX4_DIR="/workspace/px4"

# 使用说明
usage() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --gui                启用GUI模式 (Gazebo可视化)"
    echo "  --headless           启用headless模式 (无GUI)"
    echo "  --auto               自动检测模式 (默认)"
    echo "  --vehicle <name>     飞行器类型 (默认: iris)"
    echo "  --world <name>       世界环境 (默认: empty)"
    echo "  --verbose            详细输出"
    echo "  --help, -h           显示此帮助信息"
    echo ""
    echo "可用飞行器:"
    echo "  iris, plane, standard_vtol, tailsitter, typhoon_h480"
    echo ""
    echo "可用世界:"
    echo "  empty, iris, baylands, mcmillan_airfield, sonoma_raceway"
    echo ""
    echo "示例:"
    echo "  $0 --gui --vehicle iris --world empty"
    echo "  $0 --headless --vehicle plane"
    echo "  $0 --auto"
}

# 检测GUI支持
detect_gui_support() {
    if [ "$GUI_MODE" != "auto" ]; then
        return 0
    fi
    
    echo "🔍 检测GUI支持..."
    
    # 检查DISPLAY变量
    if [ -z "$DISPLAY" ]; then
        echo "ℹ️ 未设置DISPLAY，使用headless模式"
        HEADLESS=true
        return 0
    fi
    
    # 检查X11连接
    if command -v xset >/dev/null 2>&1; then
        if xset q >/dev/null 2>&1; then
            echo "✅ 检测到X11服务器，启用GUI模式"
            HEADLESS=false
        else
            echo "⚠️ X11服务器无法连接，使用headless模式"
            HEADLESS=true
        fi
    else
        echo "ℹ️ 未找到xset命令，使用headless模式"
        HEADLESS=true
    fi
}

# 检查PX4环境
check_px4_environment() {
    echo "🔍 检查PX4环境..."
    
    if [ ! -d "$PX4_DIR" ]; then
        echo "❌ PX4目录不存在: $PX4_DIR"
        echo "💡 请确保PX4已正确挂载到容器中"
        exit 1
    fi
    
    if [ ! -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
        echo "❌ PX4未编译或编译不完整"
        echo "💡 请先运行: ./scripts/build/build_px4.sh"
        exit 1
    fi
    
    echo "✅ PX4环境检查完成"
}

# 设置环境变量
setup_environment() {
    echo "🌍 设置仿真环境..."
    
    # PX4基础环境
    export PX4_HOME_LAT=39.9612
    export PX4_HOME_LON=116.3348
    export PX4_HOME_ALT=39
    export PX4_SIM_MODEL="$VEHICLE"
    export PX4_SIM_WORLD="$WORLD"
    
    # Gazebo环境
    export GAZEBO_PLUGIN_PATH="$PX4_DIR/build/px4_sitl_default/build_gazebo-classic:$GAZEBO_PLUGIN_PATH"
    export GAZEBO_MODEL_PATH="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH"
    export LD_LIBRARY_PATH="$PX4_DIR/build/px4_sitl_default/build_gazebo-classic:$LD_LIBRARY_PATH"
    
    if [ "$HEADLESS" = true ]; then
        echo "📱 配置headless模式..."
        export HEADLESS=1
        export GAZEBO_HEADLESS=1
        export QT_QPA_PLATFORM=offscreen
    else
        echo "🖥️ 配置GUI模式..."
        export QT_X11_NO_MITSHM=1
        export QT_QPA_PLATFORM=xcb
        export LIBGL_ALWAYS_INDIRECT=1
        export DISPLAY=${DISPLAY:-:0}
    fi
    
    echo "✅ 环境变量设置完成"
}

# 启动PX4仿真
start_px4_simulation() {
    echo "🚁 启动PX4 SITL仿真..."
    echo "📍 飞行器: $VEHICLE"
    echo "🌍 世界: $WORLD"
    echo "🖥️ 模式: $([ "$HEADLESS" = true ] && echo "Headless" || echo "GUI")"
    echo ""
    
    cd "$PX4_DIR" || exit 1
    
    # 根据模式选择启动命令
    if [ "$HEADLESS" = true ]; then
        echo "🔧 启动命令: make px4_sitl gazebo_${VEHICLE}_${WORLD}"
        echo ""
        
        # Headless模式启动
        if [ "$VERBOSE" = true ]; then
            make px4_sitl gazebo_${VEHICLE}_${WORLD}
        else
            make px4_sitl gazebo_${VEHICLE}_${WORLD} 2>/dev/null
        fi
    else
        echo "🔧 启动命令: make px4_sitl gazebo_${VEHICLE}_${WORLD}"
        echo ""
        echo "💡 Gazebo窗口将在几秒后启动..."
        echo "💡 按Ctrl+C停止仿真"
        echo ""
        
        # GUI模式启动
        make px4_sitl gazebo_${VEHICLE}_${WORLD}
    fi
}

# 显示连接信息
show_connection_info() {
    echo ""
    echo "🎉 PX4 SITL仿真已启动！"
    echo ""
    echo "📡 连接信息:"
    echo "   MAVLink UDP: localhost:14540"
    echo "   MAVLink TCP: localhost:4560"
    echo "   QGroundControl: 自动连接到 localhost:14540"
    echo ""
    echo "🎮 控制信息:"
    echo "   解锁: commander arm"
    echo "   起飞: commander takeoff"
    echo "   降落: commander land"
    echo "   模式切换: commander mode <manual|stabilized|offboard>"
    echo ""
    echo "🔧 调试命令:"
    echo "   PX4控制台: pxh>"
    echo "   状态查看: commander status"
    echo "   参数设置: param set <name> <value>"
    echo ""
    
    if [ "$HEADLESS" = false ]; then
        echo "🖥️ Gazebo界面:"
        echo "   - 可以拖拽视角查看3D环境"
        echo "   - 右键点击模型查看属性"
        echo "   - 使用工具栏控制仿真时间"
        echo ""
    fi
    
    echo "⚠️ 停止仿真: 按Ctrl+C"
}

# 参数解析
while [[ $# -gt 0 ]]; do
    case $1 in
        --gui)
            GUI_MODE="gui"
            HEADLESS=false
            shift
            ;;
        --headless)
            GUI_MODE="headless"
            HEADLESS=true
            shift
            ;;
        --auto)
            GUI_MODE="auto"
            shift
            ;;
        --vehicle)
            VEHICLE="$2"
            shift 2
            ;;
        --world)
            WORLD="$2"
            shift 2
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "❌ 未知参数: $1"
            usage
            exit 1
            ;;
    esac
done

# 主执行流程
main() {
    # 检测GUI支持
    detect_gui_support
    
    # 检查PX4环境
    check_px4_environment
    
    # 设置环境
    setup_environment
    
    # 显示连接信息
    show_connection_info
    
    # 启动仿真
    start_px4_simulation
}

# 信号处理
cleanup() {
    echo ""
    echo "🛑 正在停止PX4仿真..."
    pkill -f px4 >/dev/null 2>&1 || true
    pkill -f gazebo >/dev/null 2>&1 || true
    echo "✅ 仿真已停止"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 执行主函数
main

echo ""
echo "🎯 仿真已完成" 