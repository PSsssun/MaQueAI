#!/bin/bash

echo "🚀 === LIVO project intelligent compilation script (enhanced version) ==="
echo "Author: MaQueAI Team"
echo "Description: intelligent compilation of FAST-LIVO2 project, automatically handling dependency issues"
echo ""

# intelligent fix
intelligent_fix() {
  echo "🔍 === start environment detection and repair ==="

  # 1. check Sophus installation
  echo "📦 check Sophus installation status..."
  if [ ! -f "/usr/local/include/sophus/so3.hpp" ]; then
    echo "❌ Sophus header file missing, reinstall..."
    cd /opt/dependencies/Sophus
    sudo rm -rf build/
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SOPHUS_TESTS=OFF
    make -j$(nproc)
    sudo make install
    echo "✅ Sophus reinstall completed"
  else
    echo "✅ Sophus header file exists"
  fi

  # 2. check Sophus CMake configuration
  echo "🔧 check Sophus CMake configuration..."
  sophus_config_found=false

  # check multiple possible locations
  if [ -f "/usr/local/lib/cmake/Sophus/SophusConfig.cmake" ] ||
    [ -f "/usr/local/share/Sophus/SophusConfig.cmake" ] ||
    [ -f "/usr/local/cmake/Sophus/SophusConfig.cmake" ]; then
    sophus_config_found=true
  fi

  if [ "$sophus_config_found" = false ]; then
    echo "⚠️  Sophus CMake configuration file missing, regenerate..."
    cd /opt/dependencies/Sophus
    if [ -d "build" ]; then
      cd build
    else
      mkdir build && cd build
      cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SOPHUS_TESTS=OFF
      make -j$(nproc)
    fi
    sudo make install
    echo "✅ Sophus CMake configuration repaired"
  else
    echo "✅ Sophus CMake configuration normal"
  fi

  # 3. set environment variables
  echo "🌍 set compilation environment variables..."
  export CMAKE_PREFIX_PATH="/usr/local:$CMAKE_PREFIX_PATH"
  export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH"
  export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"
  export Sophus_DIR="/usr/local"
  echo "✅ environment variables set"
}

# main function
main() {
  intelligent_fix

  cd /workspace/catkin_ws/src

  # checkvikit_common
  if [ ! -d "vikit_common" ]; then
    ln -s /opt/dependencies/rpg_vikit/vikit_common ./
  fi

  cd /workspace/catkin_ws
  rm -rf build/ devel/
  source /opt/ros/noetic/setup.bash

  # 1. check current ROS environment
  echo "current ROS environment:"
  echo "ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH"
  echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"

  # 2. reset ROS environment
  echo "🔄 reset ROS environment..."
  source /opt/ros/noetic/setup.bash

  # 3. verify ROS environment
  echo "✅ verify ROS environment:"
  which catkin_make
  echo "ROS_DISTRO=$ROS_DISTRO"

  # 4. reset catkin workspace
  echo "🔧 reset catkin workspace..."
  cd /workspace/catkin_ws
  rm -rf build/ devel/

  # 5. compile an empty workspace first
  cd src
  # ensure there is CMakeLists.txt
  if [ ! -f CMakeLists.txt ]; then
    ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake CMakeLists.txt
  fi

  cd ..
  catkin_make # compile an empty workspace first

  # 6. set Sophus environment variables
  export CMAKE_PREFIX_PATH="/usr/local:$CMAKE_PREFIX_PATH"
  export Sophus_DIR="/usr/local"

  # 7. recompile
  catkin_make -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH" -DSophus_DIR="$Sophus_DIR"
}

main "$@"
