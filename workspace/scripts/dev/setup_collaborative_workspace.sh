#!/bin/bash
cd "$(dirname "$0")/.."

echo "🤝 setup MaQueAI collaborative development workspace..."
echo ""

# step 1: check Docker image
echo "📦 step 1: check Docker image..."
if ! docker images | grep -q "maqueai.*latest"; then
  echo "❌ MaQueAI Docker image not found!"
  echo "please run: ./scripts/build_docker.sh"
  exit 1
fi
echo "✅ Docker image ready"

# step 2: create host workspace
echo "🔧 step 2: setup host workspace..."

if [ ! -d "workspace" ]; then
  echo "📁 create workspace directory structure..."
  mkdir -p workspace/catkin_ws/src

  # initialize catkin workspace
  cd workspace/catkin_ws/src
  ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake CMakeLists.txt ||
    echo "# catkin workspace marker" >CMakeLists.txt
  cd ../../../

  echo "✅ host workspace directory created"
else
  echo "⚠️  workspace already exists, skip creation"
fi

# step 3: clone LIVO project to host
echo "🚀 step 3: setup LIVO development project..."
cd workspace/catkin_ws/src

if [ ! -d "LIVO" ]; then
  echo "📥 clone FAST-LIVO2 project and rename to LIVO..."
  git clone https://github.com/hku-mars/FAST-LIVO2.git LIVO || {
    echo "❌ clone failed, try other image source..."
    git clone https://gitclone.com/github.com/hku-mars/FAST-LIVO2.git LIVO || {
      echo "❌ all image sources failed, please manually clone"
      echo "manual command: git clone https://github.com/hku-mars/FAST-LIVO2.git LIVO"
    }
  }

  if [ -d "LIVO" ]; then
    echo "✅ LIVO project cloned"
  fi
else
  echo "✅ LIVO project already exists"
fi

cd ../../../

# step 4: set file permissions
echo "🔐 step 4: set file permissions..."
chmod -R 755 workspace/ 2>/dev/null || true

# step 5: initialize Git repository (for team collaboration)
echo "🌿 step 5: initialize Git repository..."
cd workspace/catkin_ws/src

if [ ! -d ".git" ]; then
  echo "🔄 initialize Git repository..."
  git init

  # create .gitignore
  cat >.gitignore <<'EOF'
# compilation generated files
build/
devel/
*.pyc
__pycache__/

# IDE files
.vscode/
.idea/
*.swp
*.swo

# system files
.DS_Store
Thumbs.db
EOF

  git add .
  git commit -m "Initial commit: LIVO workspace setup"
  echo "✅ Git repository initialized"
  echo "📝 after that, you can add remote repository: git remote add origin <your-repo-url>"
else
  echo "✅ Git repository already exists"
fi

cd ../../../

echo ""
echo "🎉 collaborative development environment setup completed!"
echo ""
echo "📋 environment structure:"
echo "✅ Docker镜像包含:"
echo "   - Ubuntu 20.04 + ROS Noetic"
echo "   - PCL, Eigen, OpenCV, Sophus (a621ff修复版), Vikit"
echo "   - 所有依赖库已预编译安装在 /opt/dependencies/"
echo ""
echo "✅ host workspace:"
echo "   - workspace/catkin_ws/src/LIVO - FAST-LIVO2 development project"
echo "   - support Git version control"
echo "   - support host IDE edit"
echo ""
echo "🚀 next steps:"
echo "1. start development container: ./scripts/run_docker_dev.sh"
echo "2. compile in container: build_livo.sh"
echo "3. check environment: check_versions.sh && check_projects.sh"
echo "4. edit in host IDE: workspace/catkin_ws/src/LIVO/"
echo ""
echo "👥 team collaboration:"
echo "1. push to remote repository: cd workspace/catkin_ws/src && git remote add origin <repo>"
echo "2. other members: git clone <repo> MaQueAI/workspace/catkin_ws/src"
echo "3. use: ./scripts/run_docker_dev.sh for development"
