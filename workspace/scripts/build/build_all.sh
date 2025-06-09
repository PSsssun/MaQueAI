#!/bin/bash

echo "🚀 === MaQueAI全量编译脚本 ==="
echo "Author: MaQueAI Team"
echo "Description: 统一编译PX4和LIVO系统"
echo ""

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 计时变量
start_time=$(date +%s)

# 日志函数
log_info() {
    echo "ℹ️  [$(date '+%H:%M:%S')] $1"
}

log_success() {
    echo "✅ [$(date '+%H:%M:%S')] $1"
}

log_error() {
    echo "❌ [$(date '+%H:%M:%S')] $1"
}

log_warning() {
    echo "⚠️  [$(date '+%H:%M:%S')] $1"
}

# 全局变量
PX4_SCRIPT=""
LIVO_SCRIPT=""

# 检查脚本是否存在
check_scripts() {
    log_info "检查构建脚本..."
    
    # 检查当前目录
    if [ -f "$SCRIPT_DIR/build_px4.sh" ]; then
        PX4_SCRIPT="$SCRIPT_DIR/build_px4.sh"
    elif [ -f "/tmp/build_scripts/build_px4.sh" ]; then
        PX4_SCRIPT="/tmp/build_scripts/build_px4.sh"
    elif [ -f "/workspace/scripts/build/build_px4.sh" ]; then
        PX4_SCRIPT="/workspace/scripts/build/build_px4.sh"
    fi
    
    if [ -f "$SCRIPT_DIR/build_livo.sh" ]; then
        LIVO_SCRIPT="$SCRIPT_DIR/build_livo.sh"
    elif [ -f "/tmp/build_scripts/build_livo.sh" ]; then
        LIVO_SCRIPT="/tmp/build_scripts/build_livo.sh"
    elif [ -f "/workspace/scripts/build/build_livo.sh" ]; then
        LIVO_SCRIPT="/workspace/scripts/build/build_livo.sh"
    fi
    
    if [ -z "$PX4_SCRIPT" ]; then
        log_error "build_px4.sh 未找到"
        exit 1
    fi
    
    if [ -z "$LIVO_SCRIPT" ]; then
        log_error "build_livo.sh 未找到"
        exit 1
    fi
    
    log_success "构建脚本检查完成: PX4=$PX4_SCRIPT, LIVO=$LIVO_SCRIPT"
}

# 构建PX4
build_px4() {
    log_info "开始构建PX4..."
    echo "================================================"
    
    if bash "$PX4_SCRIPT"; then
        log_success "PX4构建成功!"
        return 0
    else
        log_error "PX4构建失败!"
        return 1
    fi
}

# 构建LIVO
build_livo() {
    log_info "开始构建LIVO..."
    echo "================================================"
    
    if bash "$LIVO_SCRIPT"; then
        log_success "LIVO构建成功!"
        return 0
    else
        log_error "LIVO构建失败!"
        return 1
    fi
}

# 显示系统信息
show_system_info() {
    log_info "系统信息:"
    echo "   操作系统: $(lsb_release -d 2>/dev/null | cut -f2 || echo 'Unknown')"
    echo "   内核版本: $(uname -r)"
    echo "   CPU核心数: $(nproc)"
    echo "   内存信息: $(free -h | grep '^Mem:' | awk '{print $2 " total, " $7 " available"}')"
    echo "   磁盘空间: $(df -h /workspace | tail -1 | awk '{print $4 " available"}')"
}

# 主函数
main() {
    echo "🎯 开始MaQueAI全量构建流程"
    echo "⏰ 开始时间: $(date '+%Y-%m-%d %H:%M:%S')"
    echo ""
    
    # 显示系统信息
    show_system_info
    echo ""
    
    # 检查脚本
    check_scripts
    echo ""
    
    # 解析命令行参数
    SKIP_PX4=false
    SKIP_LIVO=false
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --skip-px4)
                SKIP_PX4=true
                log_warning "跳过PX4构建"
                shift
                ;;
            --skip-livo)
                SKIP_LIVO=true
                log_warning "跳过LIVO构建"
                shift
                ;;
            --px4-only)
                SKIP_LIVO=true
                log_info "仅构建PX4"
                shift
                ;;
            --livo-only)
                SKIP_PX4=true
                log_info "仅构建LIVO"
                shift
                ;;
            --help|-h)
                echo "用法: $0 [选项]"
                echo ""
                echo "选项:"
                echo "  --skip-px4      跳过PX4构建"
                echo "  --skip-livo     跳过LIVO构建"
                echo "  --px4-only      仅构建PX4"
                echo "  --livo-only     仅构建LIVO"
                echo "  --help, -h      显示帮助信息"
                echo ""
                echo "示例:"
                echo "  $0                    # 构建所有项目"
                echo "  $0 --px4-only        # 仅构建PX4"
                echo "  $0 --livo-only       # 仅构建LIVO"
                echo "  $0 --skip-px4        # 跳过PX4，仅构建LIVO"
                exit 0
                ;;
            *)
                log_warning "未知参数: $1"
                shift
                ;;
        esac
    done
    
    # 构建计数器
    build_success=0
    build_total=0
    
    # 构建PX4
    if [ "$SKIP_PX4" = false ]; then
        build_total=$((build_total + 1))
        if build_px4; then
            build_success=$((build_success + 1))
        fi
        echo ""
    fi
    
    # 构建LIVO  
    if [ "$SKIP_LIVO" = false ]; then
        build_total=$((build_total + 1))
        if build_livo; then
            build_success=$((build_success + 1))
        fi
        echo ""
    fi
    
    # 计算总耗时
    end_time=$(date +%s)
    total_time=$((end_time - start_time))
    formatted_time=$(printf "%02d:%02d:%02d" $((total_time / 3600)) $((total_time % 3600 / 60)) $((total_time % 60)))
    
    # 显示构建结果
    echo "================================================"
    echo "🎉 MaQueAI全量构建完成!"
    echo "⏰ 总耗时: $formatted_time"
    echo "📊 构建结果: $build_success/$build_total 成功"
    
    if [ $build_success -eq $build_total ] && [ $build_total -gt 0 ]; then
        log_success "所有项目构建成功!"
        echo ""
        echo "🚀 快速启动指南:"
        echo ""
        echo "🚁 PX4仿真:"
        echo "   cd /workspace/px4"
        echo "   make px4_sitl gazebo                # GUI仿真"
        echo "   HEADLESS=1 make px4_sitl gazebo     # 无头仿真"
        echo ""
        echo "🤖 LIVO系统:"
        echo "   cd /workspace"
        echo "   source /opt/ros/noetic/setup.bash"
        echo "   source catkin_ws/devel/setup.bash"
        echo "   roslaunch livo mapping_avia.launch"
        echo ""
        echo "🐳 GUI容器:"
        echo "   ./scripts/docker/run_docker_gui.sh"
        echo ""
        
        exit 0
    else
        log_error "部分项目构建失败!"
        echo ""
        echo "💡 故障排除:"
        echo "   - 检查网络连接"
        echo "   - 确保有足够的磁盘空间"
        echo "   - 查看上方的详细错误信息"
        echo "   - 尝试单独构建失败的项目"
        echo ""
        exit 1
    fi
}

# 执行主函数
main "$@" 