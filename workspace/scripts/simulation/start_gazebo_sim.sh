#!/bin/bash

echo "🌍 === Gazebo仿真环境启动器 ==="
echo "Author: MaQueAI Team"
echo "Description: 启动独立的Gazebo仿真环境，支持3D可视化和物理仿真"
echo ""

# 默认参数
WORLD="empty"
GUI_MODE=true
VERBOSE=false
PHYSICS_ENGINE="ode"
PAUSED=false

# 使用说明
usage() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --world <name>       世界环境 (默认: empty)"
    echo "  --headless           无GUI模式"
    echo "  --physics <engine>   物理引擎 (ode/bullet/dart，默认: ode)"
    echo "  --paused             暂停状态启动"
    echo "  --verbose            详细输出"
    echo "  --help, -h           显示此帮助信息"
    echo ""
    echo "可用世界:"
    echo "  empty                空世界"
    echo "  iris                 基础环境"
    echo "  warehouse            仓库环境"
    echo "  baylands             海湾地形"
    echo "  mcmillan_airfield    机场环境"
    echo "  sonoma_raceway       赛道环境"
    echo ""
    echo "示例:"
    echo "  $0 --world empty"
    echo "  $0 --world warehouse --paused"
    echo "  $0 --headless --world iris"
}

# 检测GUI支持
check_gui_support() {
    if [ "$GUI_MODE" = false ]; then
        return 0
    fi
    
    echo "🔍 检测GUI支持..."
    
    # 检查DISPLAY变量
    if [ -z "$DISPLAY" ]; then
        echo "⚠️ 未设置DISPLAY变量，强制切换到headless模式"
        GUI_MODE=false
        return 0
    fi
    
    # 检查X11连接
    if command -v xset >/dev/null 2>&1; then
        if xset q >/dev/null 2>&1; then
            echo "✅ 检测到X11服务器，启用GUI模式"
        else
            echo "⚠️ X11服务器无法连接，切换到headless模式"
            GUI_MODE=false
        fi
    else
        echo "ℹ️ 未找到xset命令，切换到headless模式"
        GUI_MODE=false
    fi
}

# 检查Gazebo环境
check_gazebo_environment() {
    echo "🔍 检查Gazebo环境..."
    
    # 检查Gazebo是否安装
    if ! command -v gazebo >/dev/null 2>&1; then
        echo "❌ Gazebo未安装或不在PATH中"
        echo "💡 请确保已安装Gazebo Classic"
        exit 1
    fi
    
    # 检查Gazebo版本
    local gazebo_version=$(gazebo --version | head -n1 | grep -o '[0-9]\+\.[0-9]\+' || echo "unknown")
    echo "ℹ️ Gazebo版本: $gazebo_version"
    
    echo "✅ Gazebo环境检查完成"
}

# 设置Gazebo环境变量
setup_gazebo_environment() {
    echo "🌍 设置Gazebo环境..."
    
    # 基础Gazebo环境
    export GAZEBO_PLUGIN_PATH="/workspace/px4/build/px4_sitl_default/build_gazebo-classic:$GAZEBO_PLUGIN_PATH"
    export GAZEBO_MODEL_PATH="/workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH"
    export LD_LIBRARY_PATH="/workspace/px4/build/px4_sitl_default/build_gazebo-classic:$LD_LIBRARY_PATH"
    
    # 物理引擎设置
    export GAZEBO_PHYSICS_ENGINE="$PHYSICS_ENGINE"
    
    if [ "$GUI_MODE" = false ]; then
        echo "📱 配置headless模式..."
        export GAZEBO_HEADLESS=1
        export QT_QPA_PLATFORM=offscreen
    else
        echo "🖥️ 配置GUI模式..."
        export QT_X11_NO_MITSHM=1
        export QT_QPA_PLATFORM=xcb
        export LIBGL_ALWAYS_INDIRECT=1
        export DISPLAY=${DISPLAY:-:0}
    fi
    
    echo "✅ Gazebo环境变量设置完成"
}

# 查找世界文件
find_world_file() {
    local world_name="$1"
    local world_file=""
    
    # 搜索路径
    local search_paths=(
        "/workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds"
        "/usr/share/gazebo-11/worlds"
        "/usr/share/gazebo/worlds"
    )
    
    for path in "${search_paths[@]}"; do
        if [ -f "$path/${world_name}.world" ]; then
            world_file="$path/${world_name}.world"
            break
        fi
    done
    
    if [ -z "$world_file" ]; then
        echo "⚠️ 未找到世界文件: ${world_name}.world"
        echo "ℹ️ 使用默认空世界"
        world_file="worlds/empty.world"
    fi
    
    echo "$world_file"
}

# 启动Gazebo
start_gazebo() {
    echo "🌍 启动Gazebo仿真环境..."
    echo "🗺️ 世界: $WORLD"
    echo "🖥️ 模式: $([ "$GUI_MODE" = true ] && echo "GUI" || echo "Headless")"
    echo "⚙️ 物理引擎: $PHYSICS_ENGINE"
    echo ""
    
    # 查找世界文件
    local world_file=$(find_world_file "$WORLD")
    echo "📂 世界文件: $world_file"
    echo ""
    
    # 构建Gazebo启动参数
    local gazebo_args=()
    
    if [ "$GUI_MODE" = false ]; then
        gazebo_args+=("--headless")
    fi
    
    if [ "$PAUSED" = true ]; then
        gazebo_args+=("--pause")
    fi
    
    if [ "$VERBOSE" = true ]; then
        gazebo_args+=("--verbose")
    fi
    
    # 添加物理引擎参数
    gazebo_args+=("--physics" "$PHYSICS_ENGINE")
    
    # 添加世界文件
    gazebo_args+=("$world_file")
    
    echo "🔧 启动命令: gazebo ${gazebo_args[*]}"
    echo ""
    
    if [ "$GUI_MODE" = true ]; then
        echo "💡 Gazebo界面即将启动..."
        echo "💡 按Ctrl+C停止仿真"
        echo ""
    fi
    
    # 启动Gazebo
    gazebo "${gazebo_args[@]}"
}

# 显示使用说明
show_usage_info() {
    echo ""
    echo "🎉 Gazebo仿真环境已启动！"
    echo ""
    if [ "$GUI_MODE" = true ]; then
        echo "🖥️ GUI界面操作说明:"
        echo "   🖱️ 鼠标左键: 选择对象"
        echo "   🖱️ 鼠标中键: 拖拽视角"
        echo "   🖱️ 鼠标右键: 旋转视角"
        echo "   ⌨️ Ctrl+R: 重置视角"
        echo "   ⌨️ Space: 暂停/继续仿真"
        echo ""
        echo "🔧 工具栏功能:"
        echo "   📦 Insert: 添加模型"
        echo "   ⚙️ World: 世界设置"
        echo "   🎮 GUI: 界面配置"
        echo ""
    fi
    
    echo "📡 网络接口:"
    echo "   Gazebo Master: http://localhost:11345"
    echo "   Topics: gazebo topic -l"
    echo "   Services: gazebo service -l"
    echo ""
    echo "⚠️ 停止仿真: 按Ctrl+C"
}

# 参数解析
while [[ $# -gt 0 ]]; do
    case $1 in
        --world)
            WORLD="$2"
            shift 2
            ;;
        --headless)
            GUI_MODE=false
            shift
            ;;
        --physics)
            PHYSICS_ENGINE="$2"
            shift 2
            ;;
        --paused)
            PAUSED=true
            shift
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
    check_gui_support
    
    # 检查Gazebo环境
    check_gazebo_environment
    
    # 设置环境
    setup_gazebo_environment
    
    # 显示使用说明
    show_usage_info
    
    # 启动Gazebo
    start_gazebo
}

# 信号处理
cleanup() {
    echo ""
    echo "🛑 正在停止Gazebo..."
    pkill -f gazebo >/dev/null 2>&1 || true
    echo "✅ Gazebo已停止"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 执行主函数
main

echo ""
echo "🎯 Gazebo仿真已完成" 