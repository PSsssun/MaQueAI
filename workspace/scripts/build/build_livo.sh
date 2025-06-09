#!/bin/bash

echo "🚀 === LIVO build script (only build LIVO) ==="
echo "Author: MaQueAI Team"
echo "Description: only build LIVO package, avoid other package problems"
echo ""

# intelligent fix
intelligent_fix() {
  echo "🔍 === check Sophus dependency ==="

  # check Sophus installation
  echo "📦 check Sophus..."
  if [ ! -f "/usr/local/include/sophus/so3.hpp" ]; then
    echo "❌ Sophus missing, reinstall..."
    cd /opt/dependencies/Sophus
    sudo rm -rf build/
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SOPHUS_TESTS=OFF
    make -j$(nproc)
    sudo make install
    echo "✅ Sophus installed"
  else
    echo "✅ Sophus exists"
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
  echo "📦 check and link vikit package..."
  for pkg in /opt/dependencies/rpg_vikit/*/; do
      if [ -f "$pkg/package.xml" ]; then
          pkg_name=$(basename "$pkg")
          if [ ! -d "$pkg_name" ]; then
              echo "  - link $pkg_name"
              ln -s "$pkg" ./
          fi
      fi
  done
  echo "✅ vikit package linked"

  # compile project
  echo "🔨 start build LIVO..."
  cd /workspace/catkin_ws
  
  # clean build directory to avoid conflict
  rm -rf build/ devel/

  # only build LIVO package and its dependencies
  echo "   build livo package..."
  if catkin_make --only-pkg-with-deps livo; then
      echo "✅ LIVO build success!"
  else
      echo "❌ LIVO build failed"
      exit 1
  fi

  echo "🎉 LIVO build completed!"
  echo "💡 usage:"
  echo "   cd /workspace/catkin_ws"
  echo "   source devel/setup.bash"
  echo "   roslaunch livo mapping_avia.launch"
}

main "$@"
