# 🚀 MaQueAI - MaQue Dockerized Development Environment

[![Docker](https://img.shields.io/badge/docker-%230db7ed.svg?style=for-the-badge&logo=docker&logoColor=white)](https://www.docker.com/)
[![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)](https://www.ros.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/)

A comprehensive, ready-to-use Docker environment for algorithm development with all dependencies pre-configured.

## ✨ Features

- 🐳 **Fully Dockerized**: Complete development environment in a container
- 🛠️ **Pre-built Dependencies**: Sophus, Vikit, PCL, Eigen, OpenCV ready to use
- 🔧 **Intelligent Build System**: Smart compilation scripts with auto-detection
- 👥 **Collaborative Development**: Workspace mounting for team collaboration
- 📊 **Build Monitoring**: Real-time Docker build progress tracking
- 🎯 **VS Code Integration**: DevContainer configuration included
- 🚀 **One-Click Setup**: Automated scripts for quick deployment


## 🔧 System Requirements

- **OS**: Linux (Ubuntu 18.04+ recommended)
- **Docker**: Version 20.0+
- **Memory**: 8GB+ RAM recommended
- **Storage**: 10GB+ free space
- **Network**: Internet connection for initial build

## 🚀 Quick Start

### 1. Clone Repository
```bash
git clone https://github.com/yourusername/MaQueAI.git
cd MaQueAI
```

### 2. Build Docker Image
```bash
./scripts/build_docker.sh
```

### 3. Start Development
Choose your preferred mode:

#### 🏠 Standalone Mode (Quick Testing)
```bash
./scripts/run_docker.sh
```

#### 👥 Development Mode (Collaborative)
```bash
./scripts/run_docker_dev.sh
```

### 4. Compile LIVO
Inside the container:
```bash
build_livo.sh
```

## 📖 Usage Guide

### Container Commands
Once inside the container, use these helpful commands:

| Command | Description |
|---------|-------------|
| `check_versions.sh` | Verify all library versions |
| `check_projects.sh` | Check project status |
| `check_dependencies.sh` | Detailed dependency info |
| `build_livo.sh` | Compile LIVO project |
| `show_structure.sh` | Display container structure |

### Running LIVO
```bash
# Source the workspace
source devel/setup.bash

# Launch the algorithm
roslaunch LIVO mapping_avia.launch

# In another terminal, play sample data
rosbag play /data/Bright_Screen_Wall.bag
```

## 🔍 Development Modes

### Mode 1: Independent Container
- **Use Case**: Quick testing, demos, learning
- **Command**: `./scripts/run_docker.sh`
- **Features**: Self-contained environment, no external dependencies

### Mode 2: Development Mount
- **Use Case**: Team collaboration, IDE integration
- **Command**: `./scripts/run_docker_dev.sh`
- **Features**: Host workspace sync, persistent changes

## 🛠️ Pre-installed Environment

| Component | Version | Status |
|-----------|---------|--------|
| Ubuntu | 20.04 LTS | ✅ |
| ROS | Noetic | ✅ |
| PCL | 1.10 | ✅ |
| Eigen | 3.3.7 | ✅ |
| OpenCV | 4.2 | ✅ |
| Sophus | a621ff (patched) | ✅ |
| Vikit | Latest | ✅ |

## 🔧 Build Monitoring

Track your Docker build progress in real-time:

```bash
# Real-time monitoring
./scripts/watch_docker_build.sh

# Build progress details
./scripts/show_build_progress.sh
```

## 🤝 Collaborative Development

### Setup Team Workspace
```bash
./scripts/setup_collaborative_workspace.sh
```

This creates a shared workspace structure perfect for team development with Git integration.

### VS Code Integration
Open the project in VS Code with DevContainer support:
1. Install "Remote - Containers" extension
2. Open project folder
3. Select "Reopen in Container"

## 🐛 Troubleshooting

### Common Issues

**Docker build fails with network errors:**
```bash
# The image includes multiple mirror sources
# Build process automatically retries with different sources
```

**Compilation errors:**
```bash
# Run the intelligent build script
build_livo.sh
# It automatically detects and fixes common dependency issues
```

**Permission errors:**
```bash
# Ensure scripts are executable
chmod +x scripts/*.sh
```

## 📂 Data Management

- Place your `.bag` files in the `data/` directory
- Files are automatically mounted to `/data` in the container
- Sample dataset `Bright_Screen_Wall.bag` included

## 🔬 Advanced Usage

### Custom Build Options
```bash
# Build with specific tag
docker build -f docker_config/Dockerfile-MaQueAI -t custom-name:tag .

# Build with build arguments
docker build --build-arg UBUNTU_VERSION=20.04 -t custom-build .
```

### Environment Variables
```bash
# Set custom display for GUI applications
export DISPLAY=:0
./scripts/run_docker_dev.sh
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📋 Roadmap

- [ ] ARM64 architecture support
- [ ] Multi-stage Docker builds for smaller images
- [ ] Automated testing pipeline
- [ ] Additional SLAM algorithms integration
- [ ] Cloud deployment templates

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) - The core SLAM algorithm
- [Sophus](https://github.com/strasdat/Sophus) - Lie algebra library
- [Vikit](https://github.com/uzh-rpg/rpg_vikit) - Vision toolkit
- ROS Community for the excellent robotics framework

## 📞 Support

If you encounter any issues or have questions:

1. Check the [Issues](https://github.com/yourusername/MaQueAI/issues) page
2. Run diagnostic commands: `check_versions.sh && check_projects.sh`
3. Create a new issue with detailed information

---

<div align="center">

**⭐ Star this repository if you find it helpful!**

Made with ❤️ by the MaQueAI Team

</div>