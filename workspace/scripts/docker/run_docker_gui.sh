#!/bin/bash

echo "🖥️ === MaQueAI GUI-enabled Docker Environment ==="
echo "start Docker container with GUI support, can run Gazebo visualization simulation"
echo ""

# calculate project root directory (MaQueAI/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# check if in the correct project root directory
if [ ! -d "$PROJECT_ROOT/workspace" ] || [ ! -d "$PROJECT_ROOT/workspace/px4" ]; then
    echo "❌ error: cannot find MaQueAI project structure"
    echo "please ensure running this script in the correct MaQueAI project directory"
    echo "📂 当前PROJECT_ROOT: $PROJECT_ROOT"
    echo "📂 查找: $PROJECT_ROOT/workspace 和 $PROJECT_ROOT/workspace/px4"
    echo "📂 SCRIPT_DIR: $SCRIPT_DIR"
    ls -la "$PROJECT_ROOT" 2>/dev/null || echo "❌ PROJECT_ROOT目录不存在"
    exit 1
fi

# check if Docker is running
if ! docker info >/dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker and try again."
    exit 1
fi

# Docker image and container name
IMAGE_NAME="maque-ai:latest"
CONTAINER_NAME="maque-ai-gui-container"

# check if Docker image exists
if ! docker images | grep -q "maque-ai"; then
    echo "❌ Docker image $IMAGE_NAME not found"
    echo "💡 please build image or ensure image name is correct"
    exit 1
fi
echo "✅ Docker image $IMAGE_NAME found"



# GUI environment detection
check_gui_support() {
    echo "🔍 check GUI support..."
    
    # check X11 server
    if [ -z "$DISPLAY" ]; then
        echo "⚠️ warning: DISPLAY environment variable not set"
        export DISPLAY=:0
        echo "DISPLAY=$DISPLAY set"
    fi
    
    # check X11 authorization
    if [ ! -f "$HOME/.Xauthority" ]; then
        echo "⚠️ warning: X11 authorization file not found, create empty file"
        touch "$HOME/.Xauthority"
    fi
    
    # check DRI device (GPU acceleration)
    GPU_DEVICE_AVAILABLE=false
    if [ -d "/dev/dri" ]; then
        echo "✅ detect GPU device, enable hardware acceleration"
        GPU_DEVICE_AVAILABLE=true
    else
        echo "ℹ️ no GPU device detected, use software rendering"
    fi
}

# prepare Docker run parameters
prepare_docker_args() {
    check_gui_support
    
    # basic parameters
    DOCKER_ARGS=(
        "--name" "$CONTAINER_NAME"
        "--rm"
        "--interactive"
        "--tty"
        "--privileged"
        "--network=host"
        
        # X11 support
        "--env" "DISPLAY=$DISPLAY"
        "--env" "QT_X11_NO_MITSHM=1"
        "--env" "QT_QPA_PLATFORM=xcb"
        "--env" "LIBGL_ALWAYS_INDIRECT=1"
        "--env" "LIBGL_ALWAYS_SOFTWARE=1"
        "--env" "MESA_GL_VERSION_OVERRIDE=3.3"
        "--env" "MESA_GLSL_VERSION_OVERRIDE=330"
        "--volume" "/tmp/.X11-unix:/tmp/.X11-unix:rw"
        "--volume" "$HOME/.Xauthority:/home/ros/.Xauthority:rw"
        
        # project directory mount
        "--volume" "$PROJECT_ROOT/workspace:/workspace"
        "--volume" "$PROJECT_ROOT:/maque-ai-root"
        
        # working directory
        "--workdir" "/workspace"
    )
    
    # add GPU support (if available)
    if [ "$GPU_DEVICE_AVAILABLE" = true ]; then
        DOCKER_ARGS+=("--device=/dev/dri")
    fi
    
    # add sound support (if device exists)
    if [ -e "/dev/snd" ]; then
        DOCKER_ARGS+=("--device" "/dev/snd")
    fi
}

# stop existing container
cleanup_existing_container() {
    if docker ps -a | grep -q "$CONTAINER_NAME"; then
        echo "🧹 clean up existing container..."
        docker stop "$CONTAINER_NAME" >/dev/null 2>&1 || true
        docker rm "$CONTAINER_NAME" >/dev/null 2>&1 || true
    fi
}

# quick start simulation
quick_start_simulation() {
    local sim_type="$1"
    echo "🚀 quick start PX4 simulation mode: $sim_type"
    
    # clean up existing container
    cleanup_existing_container
    
    # prepare Docker parameters
    prepare_docker_args
    
    # allow X11 connection
    echo "🔐 configure X11 permission..."
    xhost +local:docker >/dev/null 2>&1 || echo "⚠️ cannot set X11 permission, GUI may not work"
    
    echo "🐳 start Docker container and run simulation..."
    echo "📂 project directory: $PROJECT_ROOT"
    echo "🖥️ DISPLAY: $DISPLAY"
    echo "🎮 simulation type: $sim_type"
    echo ""
    
    # according to simulation type to select start command
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
    
    # start container and run simulation
    docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME" bash -c "$sim_cmd; bash" || {
        echo "❌ Docker container start failed"
        echo ""
        echo "💡 troubleshooting suggestions:"
        echo "   1. ensure Docker image exists: docker images | grep maque-ai"
        echo "   2. check if X11 server is running"
        echo "   3. try to run: xhost +local:docker"
        echo "   4. if in WSL, ensure X11 server is running"
        echo "   5. ensure PX4 is compiled: ls /workspace/px4/build/px4_sitl_default/bin/px4"
        exit 1
    }
    
    # restore X11 permission
    echo "🔒 restore X11 permission..."
    xhost -local:docker >/dev/null 2>&1 || true
}

# main function
main() {
    echo "🚀 start MaQueAI GUI development environment..."
    
    # clean up existing container
    cleanup_existing_container
    
    # prepare Docker parameters
    prepare_docker_args
    
    # allow X11 connection
    echo "🔐 configure X11 permission..."
    xhost +local:docker >/dev/null 2>&1 || echo "⚠️ cannot set X11 permission, GUI may not work"
    
    echo "🐳 start Docker container..."
    echo "📂 project directory: $PROJECT_ROOT"
    echo "🖥️ DISPLAY: $DISPLAY"
    echo ""
    
    # start container and show usage
    docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME" bash -c "
        echo '🎉 welcome to MaQueAI GUI development environment!'
        echo ''
        echo '📋 quick start simulation:'
        echo '   cd /workspace/px4'
        echo '   make px4_sitl gazebo           # default iris quadrotor simulation'
        echo '   make px4_sitl gazebo_plane     # fixed-wing simulation'
        echo '   make px4_sitl gazebo_vtol_standard # vertical takeoff and landing simulation'
        echo ''
        echo '🔍 check PX4 compilation status:'
        if [ -f /workspace/px4/build/px4_sitl_default/bin/px4 ]; then
            echo '   ✅ PX4 compiled, can start simulation directly'
        else
            echo '   ❌ PX4 not compiled, please run build script:'
            echo '      ./scripts/build/build_px4.sh'
        fi
        echo ''
        echo '🌍 set Gazebo environment variables...'
        export GAZEBO_MODEL_PATH=\"/workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:\$GAZEBO_MODEL_PATH\"
        export GAZEBO_PLUGIN_PATH=\"/workspace/px4/build/px4_sitl_default/build_gazebo-classic:\$GAZEBO_PLUGIN_PATH\"
        export GAZEBO_RESOURCE_PATH=\"/workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds:\$GAZEBO_RESOURCE_PATH\"
        echo '   ✅ Gazebo environment variables set'
        echo ''
        echo '💡 if first time use, please run:'
        echo '   ./scripts/build/build_px4.sh    # full build PX4 and Gazebo environment'
        echo ''
        echo '📖 input exit to exit container'
        echo '🆘 input --help to view full usage'
        echo ''
        cd /workspace
        bash
    " || {
        echo "❌ Docker container start failed"
        echo ""
        echo "💡 troubleshooting suggestions:"
        echo "   1. ensure Docker image exists: docker images | grep maque-ai"
        echo "   2. check if X11 server is running"
        echo "   3. try to run: xhost +local:docker"
        echo "   4. if in WSL, ensure X11 server is running"
        echo "   5. run build script: ./scripts/build/build_px4.sh"
        exit 1
    }
    
    # restore X11 permission
    echo "🔒 restore X11 permission..."
    xhost -local:docker >/dev/null 2>&1 || true
}

# show usage
show_usage() {
    echo "📖 GUI container usage:"
    echo ""
    echo "🖥️ start PX4 Gazebo simulation:"
    echo "   cd /workspace/px4"
    echo "   make px4_sitl gazebo                    # default iris quadrotor"
    echo "   make px4_sitl gazebo_iris               # iris quadrotor model"
    echo "   make px4_sitl gazebo_plane              # fixed-wing plane"
    echo "   make px4_sitl gazebo_vtol_standard      # vertical takeoff and landing plane"
    echo "   make px4_sitl gazebo_rover              # ground vehicle"
    echo ""
    echo "🌍 use different world environments:"
    echo "   make px4_sitl gazebo_iris__empty        # empty world"
    echo "   make px4_sitl gazebo_iris__warehouse     # warehouse world"
    echo "   make px4_sitl gazebo_iris__windy        # windy world"
    echo ""
    echo "🎮 start QGroundControl compatible mode:"
    echo "   cd /workspace && ./scripts/simulation/start_px4_sim.sh --gui"
    echo ""
    echo "🔧 compile project (include full Gazebo environment configuration):"
    echo "   ./scripts/build/build_px4.sh           # build PX4 and Gazebo environment"
    echo "   ./scripts/build/build_livo.sh          # build LIVO system"
    echo ""
    echo "📋 check all available targets:"
    echo "   make list_config_targets                # check all configuration targets"
    echo "   make px4_sitl list_vmd_make_targets     # check Gazebo model"
    echo ""
    echo "⚙️ environment variables set:"
    echo "   - DISPLAY=$DISPLAY"
    echo "   - support OpenGL hardware acceleration"
    echo "   - support audio output"
    echo "   - MAVLink端口: UDP 14540, TCP 4560"
    echo ""
}

# check parameters
if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
    show_usage
    exit 0
fi

# execute main function
main

echo ""
echo "🎉 GUI container exited"
echo "💡 restart: $0" 