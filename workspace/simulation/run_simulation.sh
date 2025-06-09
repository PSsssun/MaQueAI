#!/bin/bash

# ========================================
# MaQueAI 一键仿真测试脚本
# 在Docker容器内完整测试整个C++系统
# ========================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

# 脚本配置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
SIM_DIR="$SCRIPT_DIR"

# 默认参数
SCENARIO="basic_flight"
WORLD="empty_world"
HEADLESS=false
RECORD=false
DURATION=300  # 5分钟默认测试时间
CLEAN_START=true
AUTO_ANALYSIS=true

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}🚁 MaQueAI 一键仿真测试系统${NC}"
echo -e "${CYAN}========================================${NC}"

# ========================================
# 参数解析
# ========================================
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --scenario)
                SCENARIO="$2"
                shift 2
                ;;
            --world)
                WORLD="$2" 
                shift 2
                ;;
            --headless)
                HEADLESS=true
                shift
                ;;
            --record)
                RECORD=true
                shift
                ;;
            --duration)
                DURATION="$2"
                shift 2
                ;;
            --no-clean)
                CLEAN_START=false
                shift
                ;;
            --help|-h)
                show_help
                exit 0
                ;;
            *)
                echo -e "${RED}❌ 未知参数: $1${NC}"
                show_help
                exit 1
                ;;
        esac
    done
}

show_help() {
    echo -e "${BLUE}📖 使用说明:${NC}"
    echo -e "  $0 [选项]"
    echo -e ""
    echo -e "${BLUE}选项:${NC}"
    echo -e "  --scenario SCENE    测试场景 (basic_flight|autonomous_mission|obstacle_avoidance)"
    echo -e "  --world WORLD      仿真世界 (empty_world|indoor_complex|warehouse)"
    echo -e "  --headless         无GUI模式运行"
    echo -e "  --record           记录测试数据"
    echo -e "  --duration SEC     测试持续时间(秒)"
    echo -e "  --no-clean         不清理之前的数据"
    echo -e "  --help, -h         显示此帮助"
    echo -e ""
    echo -e "${BLUE}示例:${NC}"
    echo -e "  $0                                    # 基础测试"
    echo -e "  $0 --scenario autonomous_mission      # 自主任务测试"
    echo -e "  $0 --world indoor_complex --record    # 室内环境+数据记录"
}

# ========================================
# 环境检查
# ========================================
check_environment() {
    echo -e "\n${YELLOW}🔍 1. 环境检查...${NC}"
    
    # 检查Docker环境
    if [ ! -f /.dockerenv ]; then
        echo -e "${RED}❌ 错误: 此脚本必须在Docker容器内运行${NC}"
        echo -e "${BLUE}💡 请先启动Docker容器: ./run_docker.sh${NC}"
        exit 1
    fi
    
    # 检查ROS环境
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${YELLOW}⚠️  设置ROS环境...${NC}"
        source /opt/ros/noetic/setup.bash
    fi
    
    # 检查工作空间
    if [ ! -d "$WORKSPACE_DIR/src" ]; then
        echo -e "${RED}❌ 工作空间未找到: $WORKSPACE_DIR${NC}"
        exit 1
    fi
    
    # 检查场景文件
    if [ ! -f "$SIM_DIR/scenarios/${SCENARIO}.yaml" ]; then
        echo -e "${RED}❌ 测试场景未找到: $SCENARIO${NC}"
        echo -e "${BLUE}💡 可用场景: $(ls $SIM_DIR/scenarios/*.yaml 2>/dev/null | xargs -n1 basename -s .yaml | tr '\n' ' ')${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✅ 环境检查通过${NC}"
    echo -e "${BLUE}📍 工作空间: $WORKSPACE_DIR${NC}"
    echo -e "${BLUE}🎮 测试场景: $SCENARIO${NC}"
    echo -e "${BLUE}🌍 仿真世界: $WORLD${NC}"
    echo -e "${BLUE}⏱️  测试时长: ${DURATION}秒${NC}"
}

# ========================================
# 系统编译
# ========================================
build_system() {
    echo -e "\n${YELLOW}🔧 2. 编译MaQueAI系统...${NC}"
    
    cd "$WORKSPACE_DIR"
    
    if [ "$CLEAN_START" = true ]; then
        echo -e "${BLUE}🧹 清理旧的构建文件...${NC}"
        rm -rf build/ devel/ install/
    fi
    
    # 设置编译环境
    source /opt/ros/noetic/setup.bash
    
    echo -e "${BLUE}⚡ 高性能编译模式...${NC}"
    export CMAKE_CXX_FLAGS="-O3 -march=native -DNDEBUG"
    export MAKEFLAGS="-j$(nproc)"
    
    # 编译核心模块
    echo -e "${BLUE}📦 编译核心模块...${NC}"
    catkin_make \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS="$CMAKE_CXX_FLAGS" \
        --pkg localization planning control interface
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ 系统编译成功${NC}"
    else
        echo -e "${RED}❌ 系统编译失败${NC}"
        exit 1
    fi
    
    # 设置环境
    source devel/setup.bash
}

# ========================================
# 准备仿真环境
# ========================================
prepare_simulation() {
    echo -e "\n${YELLOW}🌍 3. 准备仿真环境...${NC}"
    
    # 创建结果目录
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    RESULT_DIR="$SIM_DIR/results/test_${SCENARIO}_${TIMESTAMP}"
    mkdir -p "$RESULT_DIR"/{logs,rosbags,screenshots,reports}
    
    echo -e "${BLUE}📁 结果目录: $RESULT_DIR${NC}"
    
    # 设置Gazebo环境
    export GAZEBO_MODEL_PATH="$SIM_DIR/models:$GAZEBO_MODEL_PATH"
    export GAZEBO_RESOURCE_PATH="$SIM_DIR/worlds:$GAZEBO_RESOURCE_PATH"
    
    # 检查显示环境
    if [ "$HEADLESS" = false ]; then
        if [ -z "$DISPLAY" ]; then
            echo -e "${YELLOW}⚠️  无显示环境，强制切换到headless模式${NC}"
            HEADLESS=true
        fi
    fi
    
    # 设置headless模式
    if [ "$HEADLESS" = true ]; then
        export DISPLAY=:99
        export LIBGL_ALWAYS_SOFTWARE=1
        # 启动虚拟显示
        Xvfb :99 -screen 0 1280x1024x24 &
        export XVFB_PID=$!
        sleep 2
    fi
    
    echo -e "${GREEN}✅ 仿真环境准备完成${NC}"
}

# ========================================
# 启动仿真组件
# ========================================
start_simulation_components() {
    echo -e "\n${YELLOW}🚀 4. 启动仿真组件...${NC}"
    
    # PID跟踪数组
    declare -a COMPONENT_PIDS=()
    
    # 1. 启动roscore
    echo -e "${BLUE}🔧 启动ROS核心...${NC}"
    roscore &
    ROSCORE_PID=$!
    COMPONENT_PIDS+=($ROSCORE_PID)
    sleep 3
    
    # 2. 启动Gazebo
    echo -e "${BLUE}🌍 启动Gazebo仿真...${NC}"
    if [ "$HEADLESS" = true ]; then
        roslaunch gazebo_ros empty_world.launch \
            world_name:="$SIM_DIR/worlds/${WORLD}.world" \
            headless:=true \
            gui:=false \
            verbose:=true &
    else
        roslaunch gazebo_ros empty_world.launch \
            world_name:="$SIM_DIR/worlds/${WORLD}.world" \
            gui:=true \
            verbose:=true &
    fi
    GAZEBO_PID=$!
    COMPONENT_PIDS+=($GAZEBO_PID)
    sleep 5
    
    # 3. 启动PX4 SITL (虚拟，在容器内运行简化版)
    echo -e "${BLUE}🛩️  启动PX4 SITL模拟...${NC}"
    # 启动MAVROS桥接到外部PX4
    roslaunch mavros px4.launch \
        fcu_url:="udp://:14540@127.0.0.1:14557" \
        gcs_url:="" &
    MAVROS_PID=$!
    COMPONENT_PIDS+=($MAVROS_PID)
    sleep 3
    
    # 4. 启动MaQueAI系统
    echo -e "${BLUE}🧠 启动MaQueAI系统...${NC}"
    roslaunch "$SIM_DIR/launch/maqueai_system.launch" \
        log_output:="file" \
        result_dir:="$RESULT_DIR" &
    MAQUEAI_PID=$!
    COMPONENT_PIDS+=($MAQUEAI_PID)
    sleep 5
    
    # 5. 启动数据记录
    if [ "$RECORD" = true ]; then
        echo -e "${BLUE}📹 启动数据记录...${NC}"
        rosbag record -a -O "$RESULT_DIR/rosbags/simulation_data.bag" &
        ROSBAG_PID=$!
        COMPONENT_PIDS+=($ROSBAG_PID)
    fi
    
    # 6. 启动可视化 (如果不是headless)
    if [ "$HEADLESS" = false ]; then
        echo -e "${BLUE}📊 启动RViz可视化...${NC}"
        roslaunch "$SIM_DIR/launch/visualization.launch" &
        RVIZ_PID=$!
        COMPONENT_PIDS+=($RVIZ_PID)
    fi
    
    echo -e "${GREEN}✅ 所有组件启动完成${NC}"
    echo -e "${BLUE}🔄 等待系统稳定...${NC}"
    sleep 10
    
    # 导出PID数组供后续使用
    export COMPONENT_PIDS_STR=$(IFS=','; echo "${COMPONENT_PIDS[*]}")
}

# ========================================
# 执行测试场景
# ========================================
execute_test_scenario() {
    echo -e "\n${YELLOW}🎮 5. 执行测试场景: $SCENARIO${NC}"
    
    # 启动场景执行器
    python3 "$SIM_DIR/scripts/scenario_executor.py" \
        --scenario "$SIM_DIR/scenarios/${SCENARIO}.yaml" \
        --duration "$DURATION" \
        --result_dir "$RESULT_DIR" \
        --verbose
    
    SCENARIO_EXIT_CODE=$?
    
    if [ $SCENARIO_EXIT_CODE -eq 0 ]; then
        echo -e "${GREEN}✅ 测试场景执行完成${NC}"
    else
        echo -e "${RED}❌ 测试场景执行失败 (退出码: $SCENARIO_EXIT_CODE)${NC}"
    fi
    
    return $SCENARIO_EXIT_CODE
}

# ========================================
# 数据收集和分析
# ========================================
collect_and_analyze() {
    echo -e "\n${YELLOW}📊 6. 数据收集和分析...${NC}"
    
    # 停止数据记录
    if [ "$RECORD" = true ] && [ -n "$ROSBAG_PID" ]; then
        echo -e "${BLUE}⏹️  停止数据记录...${NC}"
        kill $ROSBAG_PID 2>/dev/null || true
        sleep 2
    fi
    
    # 收集系统日志
    echo -e "${BLUE}📝 收集系统日志...${NC}"
    python3 "$SIM_DIR/scripts/data_collector.py" \
        --result_dir "$RESULT_DIR" \
        --ros_log_dir ~/.ros/log
    
    # 分析测试结果
    if [ "$AUTO_ANALYSIS" = true ]; then
        echo -e "${BLUE}🔍 分析测试结果...${NC}"
        python3 "$SIM_DIR/scripts/result_analyzer.py" \
            --result_dir "$RESULT_DIR" \
            --scenario "$SCENARIO" \
            --generate_report
    fi
    
    echo -e "${GREEN}✅ 数据收集和分析完成${NC}"
}

# ========================================
# 清理资源
# ========================================
cleanup() {
    echo -e "\n${YELLOW}🧹 7. 清理资源...${NC}"
    
    # 获取PID数组
    if [ -n "$COMPONENT_PIDS_STR" ]; then
        IFS=',' read -ra PIDS <<< "$COMPONENT_PIDS_STR"
        for pid in "${PIDS[@]}"; do
            if kill -0 $pid 2>/dev/null; then
                echo -e "${BLUE}🔪 终止进程: $pid${NC}"
                kill $pid 2>/dev/null || true
            fi
        done
        sleep 3
        
        # 强制终止残留进程
        for pid in "${PIDS[@]}"; do
            if kill -0 $pid 2>/dev/null; then
                kill -9 $pid 2>/dev/null || true
            fi
        done
    fi
    
    # 清理ROS环境
    rosnode kill -a 2>/dev/null || true
    pkill -f gazebo 2>/dev/null || true
    pkill -f rviz 2>/dev/null || true
    
    # 清理虚拟显示
    if [ -n "$XVFB_PID" ]; then
        kill $XVFB_PID 2>/dev/null || true
    fi
    
    echo -e "${GREEN}✅ 资源清理完成${NC}"
}

# ========================================
# 生成最终报告
# ========================================
generate_final_report() {
    echo -e "\n${YELLOW}📋 8. 生成最终报告...${NC}"
    
    local report_file="$RESULT_DIR/reports/final_report.txt"
    
    cat > "$report_file" << EOF
========================================
MaQueAI 仿真测试报告
========================================

测试时间: $(date)
测试场景: $SCENARIO
仿真世界: $WORLD
测试时长: ${DURATION}秒
Headless模式: $HEADLESS
数据记录: $RECORD

系统配置:
- ROS版本: $ROS_DISTRO
- 工作空间: $WORKSPACE_DIR
- 结果目录: $RESULT_DIR

组件状态:
$(if [ -f "$RESULT_DIR/logs/component_status.log" ]; then cat "$RESULT_DIR/logs/component_status.log"; else echo "状态日志未生成"; fi)

性能指标:
$(if [ -f "$RESULT_DIR/reports/performance_metrics.txt" ]; then cat "$RESULT_DIR/reports/performance_metrics.txt"; else echo "性能指标未生成"; fi)

测试结果:
$(if [ $SCENARIO_EXIT_CODE -eq 0 ]; then echo "✅ 测试通过"; else echo "❌ 测试失败"; fi)

========================================
EOF

    echo -e "${GREEN}📄 最终报告已生成: $report_file${NC}"
    
    # 显示摘要
    echo -e "\n${CYAN}========================================${NC}"
    echo -e "${CYAN}📊 测试摘要${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo -e "${BLUE}🎮 场景: $SCENARIO${NC}"
    echo -e "${BLUE}🌍 世界: $WORLD${NC}"
    echo -e "${BLUE}⏱️  时长: ${DURATION}秒${NC}"
    echo -e "${BLUE}📁 结果: $RESULT_DIR${NC}"
    
    if [ $SCENARIO_EXIT_CODE -eq 0 ]; then
        echo -e "${GREEN}🏆 测试状态: 通过 ✅${NC}"
    else
        echo -e "${RED}💥 测试状态: 失败 ❌${NC}"
    fi
    
    echo -e "${CYAN}========================================${NC}"
}

# ========================================
# 主函数
# ========================================
main() {
    # 设置退出陷阱
    trap cleanup EXIT
    
    # 解析参数
    parse_args "$@"
    
    # 执行测试流程
    check_environment
    build_system
    prepare_simulation
    start_simulation_components
    execute_test_scenario
    collect_and_analyze
    generate_final_report
    
    # 返回测试结果
    exit $SCENARIO_EXIT_CODE
}

# 运行主函数
main "$@" 