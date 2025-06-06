#!/bin/bash
cd "$(dirname "$0")/.."

echo "🚀 start MaQueAI Docker container (development mode - mount workspace)..."

# check and create workspace directory
if [ ! -d "workspace" ]; then
  echo "📁 create workspace directory..."
  mkdir -p workspace/catkin_ws
  echo "✅ workspace directory created"
fi

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
echo "  host: $(pwd)/workspace -> container: /workspace"
echo "  host: $(pwd)/src -> container: /workspace/catkin_ws/src"
echo "  host: $(pwd)/data -> container: /data"
echo ""
echo "💡 development mode features:"
echo "  ✅ code modified automatically synchronized to host"
echo "  ✅ support host IDE directly edit"
echo "  ✅ dependency library pre-installed (Sophus, Vikit)"
echo "  ✅ FAST-LIVO2 source code pre-loaded"
echo "  ✅ code not lost after container deletion"
echo ""
echo "🔨 to compile LIVO in container, run: build_livo.sh"
echo ""

docker run -it --rm \
  --name maqueai-dev-mounted \
  --privileged \
  --network=host \
  --env=DISPLAY=$DISPLAY \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume=$(pwd)/workspace:/workspace \
  --volume=$(pwd)/src:/workspace/catkin_ws/src \
  --volume=$(pwd)/data:/data \
  maque-ai:latest bash
