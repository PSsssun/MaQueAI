#!/bin/bash

# get script directory and calculate project root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

echo "🚀 Starting MaQueAI Docker container (using pre-built workspace)..."
echo "📍 Project root: $PROJECT_ROOT"

cd "$PROJECT_ROOT"

# check if workspace/src directory exists
if [ ! -d "workspace/src" ]; then
  echo "⚠️  workspace/src directory not found. Creating and cloning FAST-LIVO2..."
  mkdir -p workspace/src
  cd workspace/src
  git clone https://github.com/hku-mars/FAST-LIVO2.git LIVO
  cd "$PROJECT_ROOT"
  echo "✅ FAST-LIVO2 cloned to workspace/src/LIVO"
else
  echo "✅ workspace/src directory found"
  if [ ! -d "workspace/src/LIVO" ]; then
    echo "⚠️  LIVO project not found in workspace/src directory. Cloning..."
    cd workspace/src
    git clone https://github.com/hku-mars/FAST-LIVO2.git LIVO
    cd "$PROJECT_ROOT"
    echo "✅ FAST-LIVO2 cloned to workspace/src/LIVO"
  else
    echo "✅ LIVO project found in workspace/src/LIVO"
  fi
fi

# check if workspace/px4 directory exists
if [ ! -d "workspace/px4" ]; then
  echo "⚠️  PX4 project not found. Cloning PX4-Autopilot..."
  cd workspace
  git clone --recursive https://github.com/PX4/PX4-Autopilot.git px4
  cd "$PROJECT_ROOT"
  echo "✅ PX4-Autopilot cloned to workspace/px4"
else
  echo "✅ PX4 project found in workspace/px4"
fi

echo "📁 mount the following directories to the container:"
echo "  host: $(pwd)/workspace/src -> container: /workspace/catkin_ws/src"
echo "  host: $(pwd)/workspace/px4 -> container: /workspace/px4"
echo "  host: $(pwd)/workspace/scripts -> container: /workspace/scripts"
echo "  host: $(pwd)/workspace/simulation -> container: /workspace/simulation"
echo "  host: $(pwd)/data -> container: /data"
echo ""
echo "🔨 to compile LIVO in container, run: build_livo.sh"
echo "🚁 to compile PX4 in container, run: build_px4.sh"
echo ""

docker run -it --rm \
  --name maqueai-dev \
  --privileged \
  --network=host \
  --env=DISPLAY=$DISPLAY \
  --env="QT_X11_NO_MITSHM=1" \
  --env="QT_QPA_PLATFORM=xcb" \
  --env="LIBGL_ALWAYS_INDIRECT=1" \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume=$(pwd)/workspace/src:/workspace/catkin_ws/src \
  --volume=$(pwd)/workspace/px4:/workspace/px4 \
  --volume=$(pwd)/workspace/scripts:/workspace/scripts \
  --volume=$(pwd)/workspace/simulation:/workspace/simulation \
  --volume=$(pwd)/data:/data \
  maque-ai:latest bash
