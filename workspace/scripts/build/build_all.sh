#!/bin/bash

echo "🚀 === MaQueAI full build script ==="
echo "Author: MaQueAI Team"
echo "Description: build LIVO and PX4 system (LIVO first)"
echo ""

# get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# time variable
start_time=$(date +%s)

# log function
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

# global variable
PX4_SCRIPT=""
LIVO_SCRIPT=""

# check script
check_scripts() {
    log_info "check build script..."
    
    # check current directory
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
        log_error "build_px4.sh not found"
        exit 1
    fi
    
    if [ -z "$LIVO_SCRIPT" ]; then
        log_error "build_livo.sh not found"
        exit 1
    fi
    
    log_success "build script check completed: LIVO=$LIVO_SCRIPT, PX4=$PX4_SCRIPT"
}

# build LIVO
build_livo() {
    log_info "start build LIVO (priority 1)..."
    echo "================================================"
    
    if bash "$LIVO_SCRIPT"; then
        log_success "LIVO build success!"
        return 0
    else
        log_error "LIVO build failed!"
        return 1
    fi
}

# build PX4
build_px4() {
    log_info "start build PX4 (priority 2)..."
    echo "================================================"
    
    if bash "$PX4_SCRIPT"; then
        log_success "PX4 build success!"
        return 0
    else
        log_error "PX4 build failed!"
        return 1
    fi
}

# show system info
show_system_info() {
    log_info "system info:"
    echo "   OS: $(lsb_release -d 2>/dev/null | cut -f2 || echo 'Unknown')"
    echo "   Kernel: $(uname -r)"
    echo "   CPU cores: $(nproc)"
    echo "   Memory: $(free -h | grep '^Mem:' | awk '{print $2 " total, " $7 " available"}')"
    echo "   Disk space: $(df -h /workspace | tail -1 | awk '{print $4 " available"}')"
}

# main function
main() {
    echo "🎯 start MaQueAI full build process (LIVO → PX4)"
    echo "⏰ start time: $(date '+%Y-%m-%d %H:%M:%S')"
    echo ""
    
    # show system info
    show_system_info
    echo ""
    
    # check script
    check_scripts
    echo ""
    
    # parse command line arguments
    SKIP_PX4=false
    SKIP_LIVO=false
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            --skip-px4)
                SKIP_PX4=true
                log_warning "skip PX4 build"
                shift
                ;;
            --skip-livo)
                SKIP_LIVO=true
                log_warning "skip LIVO build"
                shift
                ;;
            --px4-only)
                SKIP_LIVO=true
                log_info "build PX4 only"
                shift
                ;;
            --livo-only)
                SKIP_PX4=true
                log_info "build LIVO only"
                shift
                ;;
            --help|-h)
                echo "Usage: $0 [options]"
                echo ""
                echo "Options:"
                echo "  --skip-px4      skip PX4 build"
                echo "  --skip-livo     skip LIVO build"
                echo "  --px4-only      build PX4 only"
                echo "  --livo-only     build LIVO only"
                echo "  --help, -h      show help info"
                echo ""
                echo "build order: LIVO → PX4"
                echo ""
                echo "Examples:"
                echo "  $0                    # build all projects (LIVO first)"
                echo "  $0 --px4-only        # build PX4 only"
                echo "  $0 --livo-only       # build LIVO only"
                echo "  $0 --skip-px4        # skip PX4, build LIVO only"
                exit 0
                ;;
            *)
                log_warning "unknown parameter: $1"
                shift
                ;;
        esac
    done
    
    # build counter
    build_success=0
    build_total=0
    
    # build LIVO first
    if [ "$SKIP_LIVO" = false ]; then
        build_total=$((build_total + 1))
        if build_livo; then
            build_success=$((build_success + 1))
        else
            log_warning "LIVO build failed, but continue build PX4..."
        fi
        echo ""
    fi
    
    # build PX4
    if [ "$SKIP_PX4" = false ]; then
        build_total=$((build_total + 1))
        if build_px4; then
            build_success=$((build_success + 1))
        fi
        echo ""
    fi
    
    # calculate total time
    end_time=$(date +%s)
    total_time=$((end_time - start_time))
    formatted_time=$(printf "%02d:%02d:%02d" $((total_time / 3600)) $((total_time % 3600 / 60)) $((total_time % 60)))
    
    # show build result
    echo "================================================"
    echo "🎉 MaQueAI full build completed!"
    echo "⏰ total time: $formatted_time"
    echo "📊 build result: $build_success/$build_total success"
    echo "🔄 build order: LIVO → PX4"
    
    if [ $build_success -eq $build_total ] && [ $build_total -gt 0 ]; then
        log_success "all projects build success!"
        echo ""
        echo "🚀 quick start guide:"
        echo ""
        echo "🤖 LIVO system (priority 1):"
        echo "   cd /workspace"
        echo "   source /opt/ros/noetic/setup.bash"
        echo "   source catkin_ws/devel/setup.bash"
        echo "   roslaunch livo mapping_avia.launch"
        echo ""
        echo "🚁 PX4 simulation (priority 2):"
        echo "   cd /workspace/px4"
        echo "   make px4_sitl gazebo                # GUI simulation"
        echo "   HEADLESS=1 make px4_sitl gazebo     # headless simulation"
        echo ""
        echo "🐳 GUI container:"
        echo "   ./scripts/docker/run_docker_gui.sh"
        echo ""
        
        exit 0
    else
        log_error "some projects build failed!"
        echo ""
        echo "💡 fault tolerance:"
        echo "   - check network connection"
        echo "   - ensure enough disk space"
        echo "   - check error info above"
        echo "   - try to build failed project"
        echo ""
        echo "🔄 note: build order is LIVO → PX4"
        exit 1
    fi
}

# execute main function
main "$@" 