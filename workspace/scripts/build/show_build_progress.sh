#!/bin/bash

echo "🔍 === Docker build progress monitoring ==="
echo ""

# check if Docker build is running
if pgrep -f "docker build" >/dev/null; then
  echo "✅ Docker build is running..."
  echo ""

  echo "📊 real-time monitoring options:"
  echo "1. check Docker process status"
  echo "2. monitor network usage"
  echo "3. check Docker system resource usage"
  echo "4. show recent Docker events"
  echo ""

  read -p "please select monitoring option (1-4): " choice

  case $choice in
    1)
      echo "🏗️  Docker build process:"
      ps aux | grep "docker build" | grep -v grep
      echo ""
      echo "📈 CPU and memory usage:"
      top -bn1 | grep -E "(CPU|docker)"
      ;;
    2)
      echo "🌐 network usage:"
      echo "monitoring network connections..."
      netstat -i
      echo ""
      echo "active network connections:"
      netstat -an | grep -E "(ESTABLISHED|LISTEN)" | grep -v "127.0.0.1" | head -10
      ;;
    3)
      echo "💾 Docker system resource usage:"
      docker system df
      echo ""
      echo "🔧 Docker information:"
      docker info | grep -E "(Images|Containers|Storage|Memory|CPUs)"
      ;;
    4)
      echo "📋 recent Docker events:"
      docker events --since 10m --until now
      ;;
    *)
      echo "❌ invalid choice"
      ;;
  esac

else
  echo "❌ no running Docker build process found"
  echo ""
  echo "💡 possible reasons:"
  echo "1. build not started - run ./build_docker.sh"
  echo "2. build completed - run docker images to check result"
  echo "3. build stopped - check build log"
  echo ""

  echo "📊 current Docker status:"
  docker images | head -5
  echo ""
  docker ps -a | head -5
fi

echo ""
echo "🔄 to continuously monitor, please run:"
echo "   watch -n 5 docker system df"
echo "   watch -n 2 'ps aux | grep docker'"
