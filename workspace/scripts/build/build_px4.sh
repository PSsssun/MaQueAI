#!/bin/bash

echo "🚁 === PX4 project intelligent compilation script ==="
echo "Author: MaQueAI Team"
echo "Description: intelligent compilation of PX4-Autopilot project for SITL simulation"
echo ""

# intelligent environment setup
setup_px4_environment() {
  echo "🔍 === start PX4 environment detection and setup ==="

  # 1. install PX4 dependencies
  echo "📦 check and install PX4 dependencies..."
  
  # update package list (allow failure)
  sudo apt-get update -q || echo "⚠️ apt update failed, using cached packages"
  
  # install core required packages (group install, allow partial failure)
  echo "📦 install core build tools..."
  sudo apt-get install -y \
    python3-dev \
    libc6-dev \
    libpython3-dev \
    pkg-config \
    libxml2-dev \
    libxml2-utils \
    protobuf-compiler \
    libeigen3-dev \
    libopencv-dev \
    || echo "⚠️ some core packages install failed, but continue"
  
  echo "📦 try to install Python tools (optional)..."  
  sudo apt-get install -y \
    python3-pip \
    python3-venv \
    python3-wheel \
    python3-vcstool \
    || echo "⚠️ Python tools install failed, use alternative later"
    
  echo "📦 try to install build tools (optional)..."
  sudo apt-get install -y \
    ninja-build \
    || echo "⚠️ ninja-build install failed, use make later"
    
  echo "📦 try to install GUI support (optional)..."
  sudo apt-get install -y \
    libqt5gui5 \
    libqt5opengl5-dev \
    libx11-dev \
    libxext-dev \
    libxrender-dev \
    || echo "⚠️ GUI package install failed, but not affect SITL compile"
    
  echo "📦 try to install Gazebo related (optional)..."
  sudo apt-get install -y \
    gazebo11 \
    libgazebo11-dev \
    || echo "⚠️ Gazebo package install failed, may affect 3D simulation"
  
  echo "✅ PX4 dependencies installed (some optional packages may fail)"

  # 2. setup Python environment
  echo "🐍 setup Python environment..."
  
  # check pip3 is available
  if command -v pip3 >/dev/null 2>&1; then
    echo "✅ pip3 found, installing Python packages..."
    
    # fix importlib_metadata version conflict
    echo "🔧 fix Python package version conflict..."
    pip3 install --user -U pip setuptools wheel || echo "⚠️ pip upgrade failed"
    pip3 install --user "importlib_metadata<5.0" || echo "⚠️ importlib_metadata install failed"
    
    # install PX4 required Python packages
    echo "📦 install PX4 Python dependencies..."
    pip3 install --user -U \
      kconfiglib \
      jinja2 \
      pyserial \
      cerberus \
      pyulog \
      numpy \
      toml \
      pyyaml \
      packaging \
      future \
      six \
      || echo "⚠️ some Python packages install failed"
    
    # install empy (specify version to avoid conflict)
    echo "📦 install empy..."
    pip3 install --user "empy==3.3.4" || echo "⚠️ empy install failed, try system version"
    
    # install pyros-genmsg and other PX4/Gazebo dependencies
    echo "📦 install pyros-genmsg and Gazebo Python dependencies..."
    pip3 install --user \
      pyros-genmsg \
      pymavlink \
      jsonschema \
      future \
      || echo "⚠️ some Python packages install failed"
    
    echo "✅ Python dependencies installed"
  else
    echo "⚠️ pip3 not found, trying system packages..."
    
    # try to install system version of Python packages
    sudo apt-get install -y \
      python3-empy \
      python3-numpy \
      python3-yaml \
      python3-serial \
      python3-jinja2 \
      || echo "⚠️ system Python packages install failed, but may have enough packages"
    
    echo "✅ Python environment setup completed (using system packages)"
  fi

  # 3. set environment variables for PX4
  echo "🌍 set PX4 environment variables..."
  export PX4_HOME_LAT=39.9612
  export PX4_HOME_LON=116.3348
  export PX4_HOME_ALT=39
  export PX4_SIM_MODEL=iris
  export PX4_SIM_WORLD=empty
  
  # GUI environment setup
  echo "🖥️ setup GUI environment..."
  export QT_X11_NO_MITSHM=1
  export QT_QPA_PLATFORM=xcb
  export LIBGL_ALWAYS_INDIRECT=1
  export DISPLAY=${DISPLAY:-:0}
  
  # Gazebo environment setup
  echo "🌍 setup Gazebo environment..."
  export GAZEBO_MODEL_PATH="/workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH"
  export GAZEBO_PLUGIN_PATH="/workspace/px4/build/px4_sitl_default/build_gazebo-classic:$GAZEBO_PLUGIN_PATH"
  export GAZEBO_RESOURCE_PATH="/workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds:$GAZEBO_RESOURCE_PATH"
  
  echo "✅ PX4 and Gazebo environment variables set"
}

# Gazebo validation and configuration function
setup_gazebo_environment() {
  echo "🔧 === Gazebo validation and configuration ==="
  
  # check if Gazebo is correctly installed
  if command -v gazebo >/dev/null 2>&1; then
    echo "✅ Gazebo installed: $(gazebo --version 2>/dev/null | head -1)"
  else
    echo "❌ Gazebo not found, try to reinstall..."
    sudo apt-get update -q
    sudo apt-get install -y gazebo11 libgazebo11-dev
  fi
  
  # validate Gazebo plugin path
  echo "🔍 validate Gazebo path configuration..."
  echo "   GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"
  echo "   GAZEBO_PLUGIN_PATH: $GAZEBO_PLUGIN_PATH"
  echo "   GAZEBO_RESOURCE_PATH: $GAZEBO_RESOURCE_PATH"
  
  # check if PX4 Gazebo model exists
  if [ -d "/workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models" ]; then
    echo "✅ PX4 Gazebo model directory exists"
    echo "📋 available models:"
    ls /workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models | head -5
  else
    echo "⚠️ PX4 Gazebo model directory not found, maybe need to initialize submodules"
  fi
  
  # test Gazebo basic functionality
  echo "🧪 test Gazebo basic functionality..."
  timeout 10s gazebo --version >/dev/null 2>&1 && echo "✅ Gazebo basic functionality is normal" || echo "⚠️ Gazebo test timeout or failed"
  
  echo "✅ Gazebo environment configuration completed"
}

# main function
main() {
  setup_px4_environment
  setup_gazebo_environment

  # navigate to PX4 directory
  if [ ! -d "/workspace/px4" ]; then
    echo "❌ PX4 project not found at /workspace/px4"
    echo "💡 Please ensure PX4 project is mounted or copied to the container"
    echo "   Expected location: /workspace/px4"
    echo "   This should have been copied during Docker build process"
    exit 1
  else
    echo "✅ PX4 project found at /workspace/px4"
    
    # validate PX4 project integrity
    if [ -f "/workspace/px4/Makefile" ] && [ -f "/workspace/px4/CMakeLists.txt" ]; then
      echo "✅ PX4 project structure verified"
    else
      echo "⚠️ PX4 project structure incomplete, but proceeding..."
    fi
  fi

  cd /workspace/px4
  
  echo "📂 current PX4 directory: $(pwd)"
  echo "📋 PX4 project contents:"
  ls -la | head -10

  # 1. setup submodules
  echo "📦 update submodules..."
  git submodule update --init --recursive

  # 2. run PX4 requirements installation
  echo "🔧 install PX4 toolchain requirements..."
  
  # set environment variables to avoid interactive installation
  export DEBIAN_FRONTEND=noninteractive
  export PYTHONPATH="/home/ros/.local/lib/python3.8/site-packages:$PYTHONPATH"
  export PATH="/home/ros/.local/bin:$PATH"
  
  # install GStreamer development package (solve Gazebo compile problem)
  echo "📦 install GStreamer dependencies..."
  sudo apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav
  
  # skip problematic installation, only install necessary
  echo "📦 install PX4 toolchain (skip simulation tools)..."
  bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools || {
    echo "⚠️  ubuntu.sh install failed, manually install critical dependencies..."
    
    # manually install critical tools
    sudo apt-get install -y \
      build-essential \
      cmake \
      git \
      ninja-build \
      ccache \
      astyle \
      genromfs \
      zip \
      python3-empy \
      python3-toml \
      python3-numpy \
      python3-yaml \
      python3-dev \
      python3-pip
  }

  # 3. build PX4 for SITL simulation
  echo "🚁 building PX4 for SITL simulation..."
  
  # validate Python and critical packages
  echo "🔍 validate Python environment..."
  python3 --version
  echo "Python path: $(which python3)"
  echo "PYTHONPATH: $PYTHONPATH"
  echo "PATH: $PATH"
  
  # try to fix empy module problem
  echo "🔧 fix Python module path..."
  python3 -c "import sys; print('Python sys.path:', sys.path)" || true
  
  # reinstall empy to correct location
  echo "📦 reinstall empy to system path..."
  pip3 install --user --force-reinstall empy==3.3.4
  
  python3 -c "import kconfiglib; print('kconfiglib: OK')" || echo "❌ kconfiglib not installed"
  python3 -c "import empy; print('empy: OK')" || {
    echo "❌ empy not found, try to install..."
    sudo apt-get install -y python3-empy
    python3 -c "import empy; print('empy (system version): OK')" || echo "❌ empy not found"
  }
  
  # clean and reconfigure
  make clean || true
  make distclean || true
  
  # build basic SITL target
  echo "🔨 build PX4 SITL basic target..."
  if make px4_sitl_default; then
    echo "✅ PX4 SITL basic build success"
  else
    echo "❌ PX4 SITL basic build failed"
    exit 1
  fi
  
  # validate Gazebo environment variables
  echo "🔍 validate Gazebo environment..."
  export GAZEBO_MODEL_PATH="/workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH"
  export GAZEBO_PLUGIN_PATH="/workspace/px4/build/px4_sitl_default/build_gazebo-classic:$GAZEBO_PLUGIN_PATH"
  export GAZEBO_RESOURCE_PATH="/workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds:$GAZEBO_RESOURCE_PATH"
  
  # build Gazebo simulation target
  echo "🌍 build PX4 Gazebo simulation support..."
  if make px4_sitl gazebo; then
    echo "✅ PX4 SITL Gazebo build success!"
    echo ""
    echo "🎉 build completed! support full Gazebo visualization simulation!"
    echo ""
    echo "💡 usage:"
    echo "   🖥️  GUI mode (Gazebo visualization):"
    echo "      cd /workspace/px4"
    echo "      make px4_sitl gazebo                    # default iris quadrotor"
    echo "      make px4_sitl gazebo_iris               # iris quadrotor"
    echo "      make px4_sitl gazebo_plane              # fixed-wing plane"
    echo "      make px4_sitl gazebo_vtol_standard      # vertical takeoff and landing plane"
    echo "      make px4_sitl gazebo_rover              # ground vehicle"
    echo ""
    echo "   🌍 different world environments:"
    echo "      make px4_sitl gazebo_iris__empty        # empty world"
    echo "      make px4_sitl gazebo_iris__warehouse     # warehouse world"
    echo "      make px4_sitl gazebo_iris__windy        # windy world"
    echo ""
    echo "   📱 Headless mode (no GUI background simulation):"
    echo "      cd /workspace/px4"
    echo "      HEADLESS=1 make px4_sitl gazebo"
    echo ""
    echo "   🔧 manually start:"
    echo "      cd /workspace/px4"
    echo "      ./build/px4_sitl_default/bin/px4 -s etc/init.d-posix/rcS"
    echo ""
    echo "   📡 connection information:"
    echo "      - MAVLink UDP: localhost:14540"
    echo "      - QGroundControl: automatically connect localhost:14540"
    echo "      - MAVSDK: tcp://localhost:4560"
    echo ""
    echo "   🚀 quick start script:"
    echo "      ./scripts/simulation/start_px4_sim.sh --gui"
    echo "      ./scripts/simulation/start_px4_sim.sh --headless"
    echo ""
    echo "   🔍 view available targets:"
    echo "      make list_config_targets                # all configuration targets"
    echo "      make px4_sitl list_vmd_make_targets     # Gazebo model targets"
    
  else
    echo "⚠️  PX4 SITL gazebo build failed, trying basic SITL..."
    
    # clean failed build
    make clean || true
    make distclean || true
    
    # try to compile basic SITL (without Gazebo)
    if make px4_sitl; then
      echo "✅ PX4 basic SITL build succeeded!"
      echo ""
      echo "💡 Basic SITL usage:"
      echo "   cd /workspace/px4"
      echo "   make px4_sitl none_iris"
    else
      echo "❌ PX4 SITL build failed"
      exit 1
    fi
  fi
}

main "$@" 