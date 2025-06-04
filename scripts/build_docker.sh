#!/bin/bash
cd "$(dirname "$0")/.."

echo "🏗️  === MaQueAI Docker image build ==="
echo "📦  image name: maque-ai:latest"
echo "📂  build directory: $(pwd)"
echo "⏰  start time: $(date '+%Y-%m-%d %H:%M:%S')"
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
        elapsed_formatted=$(printf "%02d:%02d:%02d" $((elapsed/3600)) $((elapsed%3600/60)) $((elapsed%60)))
        
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
    echo "   ./run_docker.sh      # start container"
    echo "   ./build_livo.sh      # compile LIVO project"
else
    echo "❌  build failed!"
    echo "⏱️  build time: ${build_duration} seconds"
    echo "🔍  please check the error information above"
    exit 1
fi 