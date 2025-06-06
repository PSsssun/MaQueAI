#!/bin/bash
cd "$(dirname "$0")/.."

echo "🚀 Starting MaQueAI Docker container (using pre-built workspace)..."

# check if src directory exists
if [ ! -d "src" ]; then
  echo "⚠️  src directory not found. Creating and cloning FAST-LIVO2..."
  mkdir -p src
  cd src
  git clone https://github.com/hku-mars/FAST-LIVO2.git LIVO
  cd ..
  echo "✅ FAST-LIVO2 cloned to src/LIVO"
else
  echo "✅ src directory found"
  if [ ! -d "src/LIVO" ]; then
    echo "⚠️  LIVO project not found in src directory. Cloning..."
    cd src
    git clone https://github.com/hku-mars/FAST-LIVO2.git LIVO
    cd ..
    echo "✅ FAST-LIVO2 cloned to src/LIVO"
  else
    echo "✅ LIVO project found in src/LIVO"
  fi
fi

echo "📁 mount the following directories to the container:"
echo "  host: $(pwd)/src -> container: /workspace/catkin_ws/src"
echo "  host: $(pwd)/data -> container: /data"
echo ""
echo "🔨 to compile LIVO in container, run: build_livo.sh"
echo ""

docker run -it --rm \
  --name maqueai-dev \
  --privileged \
  --network=host \
  --env=DISPLAY=$DISPLAY \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume=$(pwd)/src:/workspace/catkin_ws/src \
  --volume=$(pwd)/data:/data \
  maque-ai:latest bash
