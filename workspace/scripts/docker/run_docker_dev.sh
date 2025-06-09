#!/bin/bash

# calculate project root directory (MaQueAI/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

echo "🚀 start MaQueAI Docker container (development mode - mount workspace)..."
echo "📍 Project root: $PROJECT_ROOT"

cd "$PROJECT_ROOT"

# check and create workspace directory
if [ ! -d "workspace" ]; then
  echo "📁 create workspace directory..."
  mkdir -p workspace/src
  echo "✅ workspace directory created"
fi

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
echo "  host: $(pwd)/workspace -> container: /workspace"
echo "  host: $(pwd)/data -> container: /data"
echo ""
echo "💡 development mode features:"
echo "  ✅ code modified automatically synchronized to host"
echo "  ✅ support host IDE directly edit"
echo "  ✅ dependency library pre-installed (Sophus, Vikit)"
echo "  ✅ FAST-LIVO2 source code pre-loaded"
echo "  ✅ PX4-Autopilot source code pre-loaded"
echo "  ✅ code not lost after container deletion"
echo ""
echo "🔨 to compile LIVO in container, run: build_livo.sh"
echo "🚁 to compile PX4 in container, run: build_px4.sh"
echo ""

docker run -it --rm \
  --name maqueai-dev-mounted \
  --privileged \
  --network=host \
  --env=DISPLAY=$DISPLAY \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume=$(pwd)/workspace:/workspace \
  --volume=$(pwd)/data:/data \
  maque-ai:latest bash
