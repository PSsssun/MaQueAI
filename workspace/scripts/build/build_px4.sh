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
  
  # update package list (允许失败)
  sudo apt-get update -q || echo "⚠️ apt update failed, using cached packages"
  
  # 安装核心必需包（分组安装，允许部分失败）
  echo "📦 安装核心构建工具..."
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
    || echo "⚠️ 部分核心包安装失败，但继续进行"
  
  echo "📦 尝试安装Python工具（可选）..."  
  sudo apt-get install -y \
    python3-pip \
    python3-venv \
    python3-wheel \
    python3-vcstool \
    || echo "⚠️ Python工具安装失败，稍后使用替代方案"
    
  echo "📦 尝试安装构建工具（可选）..."
  sudo apt-get install -y \
    ninja-build \
    || echo "⚠️ ninja-build安装失败，将使用make"
    
  echo "📦 尝试安装GUI支持（可选）..."
  sudo apt-get install -y \
    libqt5gui5 \
    libqt5opengl5-dev \
    libx11-dev \
    libxext-dev \
    libxrender-dev \
    || echo "⚠️ GUI包安装失败，但不影响SITL编译"
    
  echo "📦 尝试安装Gazebo相关（可选）..."
  sudo apt-get install -y \
    gazebo11 \
    libgazebo11-dev \
    || echo "⚠️ Gazebo包安装失败，可能影响3D仿真"
  
  echo "✅ PX4依赖安装完成（部分可选包可能失败）"

  # 2. setup Python environment
  echo "🐍 setup Python environment..."
  
  # 检查pip3是否可用
  if command -v pip3 >/dev/null 2>&1; then
    echo "✅ pip3 found, installing Python packages..."
    
    # 修复importlib_metadata版本冲突
    echo "🔧 修复Python包版本冲突..."
    pip3 install --user -U pip setuptools wheel || echo "⚠️ pip upgrade failed"
    pip3 install --user "importlib_metadata<5.0" || echo "⚠️ importlib_metadata install failed"
    
    # 安装PX4必需的Python包
    echo "📦 安装PX4 Python依赖..."
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
      || echo "⚠️ 部分Python包安装失败"
    
    # 安装empy (指定版本避免冲突)
    echo "📦 安装empy..."
    pip3 install --user "empy==3.3.4" || echo "⚠️ empy安装失败，尝试系统版本"
    
    # 安装pyros-genmsg和其他PX4/Gazebo依赖
    echo "📦 安装pyros-genmsg和Gazebo Python依赖..."
    pip3 install --user \
      pyros-genmsg \
      pymavlink \
      jsonschema \
      future \
      || echo "⚠️ 部分Python包安装失败"
    
    echo "✅ Python dependencies installed"
  else
    echo "⚠️ pip3 not found, trying system packages..."
    
    # 尝试安装系统版本的Python包
    sudo apt-get install -y \
      python3-empy \
      python3-numpy \
      python3-yaml \
      python3-serial \
      python3-jinja2 \
      || echo "⚠️ 系统Python包安装失败，但可能已有足够的包"
    
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

# Gazebo验证和配置函数
setup_gazebo_environment() {
  echo "🔧 === Gazebo环境验证和配置 ==="
  
  # 检查Gazebo是否正确安装
  if command -v gazebo >/dev/null 2>&1; then
    echo "✅ Gazebo已安装: $(gazebo --version 2>/dev/null | head -1)"
  else
    echo "❌ Gazebo未找到，尝试重新安装..."
    sudo apt-get update -q
    sudo apt-get install -y gazebo11 libgazebo11-dev
  fi
  
  # 验证Gazebo插件路径
  echo "🔍 验证Gazebo路径配置..."
  echo "   GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"
  echo "   GAZEBO_PLUGIN_PATH: $GAZEBO_PLUGIN_PATH"
  echo "   GAZEBO_RESOURCE_PATH: $GAZEBO_RESOURCE_PATH"
  
  # 检查PX4 Gazebo模型是否存在
  if [ -d "/workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models" ]; then
    echo "✅ PX4 Gazebo模型目录存在"
    echo "📋 可用模型:"
    ls /workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models | head -5
  else
    echo "⚠️ PX4 Gazebo模型目录不存在，可能需要初始化子模块"
  fi
  
  # 测试Gazebo基本功能
  echo "🧪 测试Gazebo基本功能..."
  timeout 10s gazebo --version >/dev/null 2>&1 && echo "✅ Gazebo基本功能正常" || echo "⚠️ Gazebo测试超时或失败"
  
  echo "✅ Gazebo环境配置完成"
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
    
    # 验证PX4项目的完整性
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
  
  # 设置环境变量避免交互式安装
  export DEBIAN_FRONTEND=noninteractive
  export PYTHONPATH="/home/ros/.local/lib/python3.8/site-packages:$PYTHONPATH"
  export PATH="/home/ros/.local/bin:$PATH"
  
  # 安装GStreamer开发包（解决Gazebo编译问题）
  echo "📦 安装GStreamer依赖..."
  sudo apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav
  
  # 跳过有问题的安装，只安装必要的
  echo "📦 安装PX4工具链（跳过仿真工具）..."
  bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools || {
    echo "⚠️  ubuntu.sh安装部分失败，手动安装关键依赖..."
    
    # 手动安装关键工具
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
  
  # 验证Python和关键包
  echo "🔍 验证Python环境..."
  python3 --version
  echo "Python路径: $(which python3)"
  echo "PYTHONPATH: $PYTHONPATH"
  echo "PATH: $PATH"
  
  # 尝试修复empy模块问题
  echo "🔧 修复Python模块路径..."
  python3 -c "import sys; print('Python sys.path:', sys.path)" || true
  
  # 重新安装empy到正确位置
  echo "📦 重新安装empy到系统路径..."
  pip3 install --user --force-reinstall empy==3.3.4
  
  python3 -c "import kconfiglib; print('kconfiglib: OK')" || echo "❌ kconfiglib未安装"
  python3 -c "import empy; print('empy: OK')" || {
    echo "❌ empy仍然未找到，尝试系统安装..."
    sudo apt-get install -y python3-empy
    python3 -c "import empy; print('empy (系统版本): OK')" || echo "❌ empy完全无法找到"
  }
  
  # 清理并重新配置
  make clean || true
  make distclean || true
  
  # 首先构建基本的SITL目标
  echo "🔨 构建PX4 SITL基础目标..."
  if make px4_sitl_default; then
    echo "✅ PX4 SITL基础构建成功"
  else
    echo "❌ PX4 SITL基础构建失败"
    exit 1
  fi
  
  # 验证Gazebo环境变量设置
  echo "🔍 验证Gazebo环境..."
  export GAZEBO_MODEL_PATH="/workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH"
  export GAZEBO_PLUGIN_PATH="/workspace/px4/build/px4_sitl_default/build_gazebo-classic:$GAZEBO_PLUGIN_PATH"
  export GAZEBO_RESOURCE_PATH="/workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds:$GAZEBO_RESOURCE_PATH"
  
  # 构建Gazebo仿真目标
  echo "🌍 构建PX4 Gazebo仿真支持..."
  if make px4_sitl gazebo; then
    echo "✅ PX4 SITL Gazebo构建成功!"
    echo ""
    echo "🎉 构建完成！支持完整的Gazebo可视化仿真!"
    echo ""
    echo "💡 使用说明:"
    echo "   🖥️  GUI模式 (Gazebo可视化):"
    echo "      cd /workspace/px4"
    echo "      make px4_sitl gazebo                    # 默认iris四旋翼"
    echo "      make px4_sitl gazebo_iris               # iris四旋翼"
    echo "      make px4_sitl gazebo_plane              # 固定翼飞机"
    echo "      make px4_sitl gazebo_vtol_standard      # 垂直起降飞机"
    echo "      make px4_sitl gazebo_rover              # 地面车辆"
    echo ""
    echo "   🌍 不同世界环境:"
    echo "      make px4_sitl gazebo_iris__empty        # 空白世界"
    echo "      make px4_sitl gazebo_iris__warehouse     # 仓库世界"
    echo "      make px4_sitl gazebo_iris__windy        # 有风环境"
    echo ""
    echo "   📱 Headless模式 (无GUI后台仿真):"
    echo "      cd /workspace/px4"
    echo "      HEADLESS=1 make px4_sitl gazebo"
    echo ""
    echo "   🔧 手动启动:"
    echo "      cd /workspace/px4"
    echo "      ./build/px4_sitl_default/bin/px4 -s etc/init.d-posix/rcS"
    echo ""
    echo "   📡 连接信息:"
    echo "      - MAVLink UDP: localhost:14540"
    echo "      - QGroundControl: 自动连接 localhost:14540"
    echo "      - MAVSDK: tcp://localhost:4560"
    echo ""
    echo "   🚀 快速启动脚本:"
    echo "      ./scripts/simulation/start_px4_sim.sh --gui"
    echo "      ./scripts/simulation/start_px4_sim.sh --headless"
    echo ""
    echo "   🔍 查看可用目标:"
    echo "      make list_config_targets                # 所有配置目标"
    echo "      make px4_sitl list_vmd_make_targets     # Gazebo模型目标"
    
  else
    echo "⚠️  PX4 SITL gazebo build failed, trying basic SITL..."
    
    # 清理失败的构建
    make clean || true
    make distclean || true
    
    # 尝试只编译基本SITL（无Gazebo）
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