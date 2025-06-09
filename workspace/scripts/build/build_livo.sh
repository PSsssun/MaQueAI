#!/bin/bash

echo "🚀 === LIVO编译脚本 (只编译LIVO) ==="
echo "Author: MaQueAI Team"
echo "Description: 只编译LIVO包，避免其他包的问题"
echo ""

# intelligent fix
intelligent_fix() {
  echo "🔍 === 检查Sophus依赖 ==="

  # check Sophus installation
  echo "📦 检查Sophus..."
  if [ ! -f "/usr/local/include/sophus/so3.hpp" ]; then
    echo "❌ Sophus缺失，重新安装..."
    cd /opt/dependencies/Sophus
    sudo rm -rf build/
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SOPHUS_TESTS=OFF
    make -j$(nproc)
    sudo make install
    echo "✅ Sophus安装完成"
  else
    echo "✅ Sophus存在"
  fi

  # set environment variables
  export CMAKE_PREFIX_PATH="/usr/local:$CMAKE_PREFIX_PATH"
  export Sophus_DIR="/usr/local"
}

# main function
main() {
  intelligent_fix

  # set ROS environment
  source /opt/ros/noetic/setup.bash

  # enter workspace
  cd /workspace/catkin_ws/src

  # check LIVO project
  if [ ! -d "LIVO" ]; then
      echo "❌ LIVO project does not exist, cloning..."
      git clone https://github.com/hku-mars/FAST-LIVO2.git LIVO || {
          echo "⚠️  GitHub cloning failed, trying mirror source..."
          git clone https://gitclone.com/github.com/hku-mars/FAST-LIVO2.git LIVO || {
              echo "❌ all sources failed, please clone manually"
              exit 1
          }
      }
      echo "✅ LIVO project cloned"
  else
      echo "✅ LIVO project exists"
  fi

  # link vikit package (check all sub-packages)
  echo "📦 检查并链接vikit包..."
  for pkg in /opt/dependencies/rpg_vikit/*/; do
      if [ -f "$pkg/package.xml" ]; then
          pkg_name=$(basename "$pkg")
          if [ ! -d "$pkg_name" ]; then
              echo "  - 链接 $pkg_name"
              ln -s "$pkg" ./
          fi
      fi
  done
  echo "✅ vikit包已链接"

  # compile project
  echo "🔨 开始编译LIVO..."
  cd /workspace/catkin_ws
  
  # 清理构建目录避免冲突
  rm -rf build/ devel/

  # 只编译LIVO包和其依赖
  echo "   编译livo包..."
  if catkin_make --only-pkg-with-deps livo; then
      echo "✅ LIVO编译成功!"
  else
      echo "❌ LIVO编译失败"
      exit 1
  fi

  echo "🎉 LIVO编译完成!"
  echo "💡 使用方法:"
  echo "   cd /workspace/catkin_ws"
  echo "   source devel/setup.bash"
  echo "   roslaunch livo mapping_avia.launch"
}

main "$@"
