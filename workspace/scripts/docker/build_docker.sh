#!/bin/bash

# 计算项目根目录 (MaQueAI/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# 切换到项目根目录进行构建
cd "$PROJECT_ROOT"

echo "🏗️  === MaQueAI Docker image build ==="
echo "📦  image name: maque-ai:latest"
echo "📂  project root: $PROJECT_ROOT"
echo "📂  build directory: $(pwd)"
echo "📄  dockerfile: $(ls -la docker_config/Dockerfile-MaQueAI 2>/dev/null && echo "✅ found" || echo "❌ not found")"
echo "⏰  start time: $(date '+%Y-%m-%d %H:%M:%S')"
echo ""

# 验证必要文件是否存在
if [ ! -f "docker_config/Dockerfile-MaQueAI" ]; then
    echo "❌ 错误: 未找到 Dockerfile"
    echo "📂 当前目录: $(pwd)"
    echo "📂 预期文件: docker_config/Dockerfile-MaQueAI"
    echo "💡 请确保从项目根目录运行此脚本"
    exit 1
fi

if [ ! -d "workspace/scripts/build" ]; then
    echo "❌ 错误: 未找到构建脚本目录"
    echo "📂 当前目录: $(pwd)"  
    echo "📂 预期目录: workspace/scripts/build"
    exit 1
fi

echo "✅ 环境验证通过"
echo ""

# record start time
start_time=$(date +%s)

# display build parameters
echo "🔧  build parameters:"
echo "   - show detailed progress: ✅"
echo "   - use build cache: ✅"
echo "   - parallel build: ✅"
echo "   - real-time timestamp: ✅"
echo ""

echo "🚀  start building..."
echo "======================================"

# use detailed progress display, and add real-time timestamp
docker build \
  --progress=plain \
  -f docker_config/Dockerfile-MaQueAI \
  -t maque-ai:latest \
  . 2>&1 | while IFS= read -r line; do
  current_time=$(date +%s)
  elapsed=$((current_time - start_time))
  time_stamp=$(date '+%H:%M:%S')
  elapsed_formatted=$(printf "%02d:%02d:%02d" $((elapsed / 3600)) $((elapsed % 3600 / 60)) $((elapsed % 60)))

  # parse Docker progress information
  if [[ "$line" =~ \#[0-9]+\ [0-9]+\.[0-9]+ ]]; then
    step_time=$(echo "$line" | grep -o '[0-9]*\.[0-9]*' | head -1)
    echo "⏱️ [$time_stamp|total time:$elapsed_formatted|step:${step_time}s] $line"
  else
    echo "⏱️ [$time_stamp|total time:$elapsed_formatted] $line"
  fi
done

# check build result
build_result=${PIPESTATUS[0]}
end_time=$(date +%s)
build_duration=$((end_time - start_time))

echo ""
echo "======================================"

if [ $build_result -eq 0 ]; then
  echo "✅  build succeeded!"
  echo "⏱️  total build time: ${build_duration} seconds ($(date -u -d @${build_duration} +'%H:%M:%S'))"
  echo "🎉  end time: $(date '+%Y-%m-%d %H:%M:%S')"
  echo ""
  echo "📊  image information:"
  docker images | head -1
  docker images | grep maque-ai
  echo ""
  echo "💡  usage:"
echo "   ./workspace/scripts/docker/run_docker_gui.sh    # start GUI container"
echo "   ./workspace/scripts/build/build_all.sh          # compile all projects" 
echo "   ./workspace/scripts/build/build_px4.sh          # compile PX4 only"
echo "   ./workspace/scripts/build/build_livo.sh         # compile LIVO only"
else
  echo "❌  build failed!"
  echo "⏱️  build time: ${build_duration} seconds"
  echo "🔍  please check the error information above"
  exit 1
fi
