#!/bin/bash
cd "$(dirname "$0")/.."

echo "🚀 Starting MaQueAI Docker container (using pre-built workspace)..."

docker run -it --rm \
  --name maqueai-dev \
  --privileged \
  --network=host \
  --env=DISPLAY=$DISPLAY \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume=$(pwd)/data:/data \
  maqueai-ros:latest bash 