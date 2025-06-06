#!/bin/bash
cd "$(dirname "$0")/.."

echo "🚀 start MaQueAI Docker container (development mode - mount workspace)..."

# check and create workspace directory
if [ ! -d "workspace" ]; then
  echo "📁 create workspace directory..."
  mkdir -p workspace/catkin_ws/src
  echo "✅ workspace directory created"
  echo "⚠️  first time use, please run the following commands in the container:"
  echo "   1. cd /workspace/catkin_ws/src"
  echo "   2. git clone https://github.com/hku-mars/FAST-LIVO2.git LIVO"
  echo "   3. cd .. && catkin_make"
fi

echo "📁 mount the following directories to the container:"
echo "  host: $(pwd)/workspace -> container: /workspace"
echo "  host: $(pwd)/data -> container: /data"
echo ""
echo "💡 development mode features:"
echo "  ✅ code modified automatically synchronized to host"
echo "  ✅ support host IDE directly edit"
echo "  ✅ dependency library pre-installed (Sophus, Vikit)"
echo "  ✅ code not lost after container deletion"
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
