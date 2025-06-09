# 🛠️ MaQueAI Scripts 脚本分类指南

## 📁 目录结构

```
scripts/
├── 📄 README.md                    # 本说明文档
│
├── 📁 build/                       # 构建相关脚本
│   ├── 🔨 build_all.sh            # 完整系统构建
│   ├── 🗺️ build_livo.sh           # LIVO SLAM构建
│   └── 🚁 build_px4.sh            # PX4飞控构建
│
├── 📁 docker/                      # Docker容器管理
│   ├── 🐳 run_docker.sh           # 启动标准容器
│   ├── 🔧 run_docker_dev.sh       # 启动开发容器
│   ├── 🏗️ build_docker.sh         # 构建Docker镜像
│   └── 👀 watch_docker_build.sh   # 监控Docker构建
│
├── 📁 dev/                         # 开发工具
│   ├── 🎨 run_format.sh           # 代码格式化
│   ├── ✅ verify_setup.sh         # 环境验证
│   └── 🏗️ setup_collaborative_workspace.sh  # 协作环境设置
│
├── 📁 simulation/                  # 仿真相关
│   └── 🚁 start_px4_simulation.sh # 启动PX4仿真
│
└── 📁 utils/                       # 实用工具
    ├── 📖 explain_structure.sh    # 项目结构说明
    └── 📊 show_build_progress.sh  # 构建进度显示
```

## 🚀 快速使用指南

### 🐳 Docker 容器管理

```bash
# 启动开发容器 (推荐)
./scripts/docker/run_docker_dev.sh

# 启动标准容器
./scripts/docker/run_docker.sh

# 构建Docker镜像
./scripts/docker/build_docker.sh
```

### 🔨 系统构建

```bash
# 在Docker容器内运行:

# 构建完整系统
./scripts/build/build_all.sh

# 只构建LIVO
./scripts/build/build_livo.sh

# 只构建PX4
./scripts/build/build_px4.sh
```

### 🚁 仿真测试

```bash
# 启动PX4仿真 (在宿主机)
./scripts/simulation/start_px4_simulation.sh

# 启动完整仿真测试 (在容器内)
./simulation/run_simulation.sh
```

### 🔧 开发工具

```bash
# 代码格式化
./scripts/dev/run_format.sh

# 验证环境设置
./scripts/dev/verify_setup.sh

# 设置协作环境
./scripts/dev/setup_collaborative_workspace.sh
```

## 📋 脚本详细说明

### 🔨 构建脚本 (build/)

#### `build_all.sh` - 完整系统构建
- **功能**: 构建LIVO + MAVROS + PX4完整系统
- **用途**: 一键构建所有组件
- **运行环境**: Docker容器内
- **依赖**: ROS Noetic, 所有依赖库

#### `build_livo.sh` - LIVO SLAM构建  
- **功能**: 智能构建FAST-LIVO2 SLAM系统
- **特性**: 自动修复Sophus依赖问题
- **运行环境**: Docker容器内
- **输出**: catkin工作空间中的LIVO包

#### `build_px4.sh` - PX4飞控构建
- **功能**: 构建PX4 SITL仿真版本
- **用途**: 为仿真准备PX4固件
- **运行环境**: Docker容器内或宿主机
- **输出**: px4_sitl可执行文件

### 🐳 Docker脚本 (docker/)

#### `run_docker.sh` - 标准容器
- **功能**: 启动预构建的MaQueAI容器
- **挂载**: 只挂载src和px4目录
- **用途**: 快速测试和运行

#### `run_docker_dev.sh` - 开发容器
- **功能**: 启动开发模式容器
- **挂载**: 完整workspace目录
- **特性**: 代码实时同步，支持IDE编辑

#### `build_docker.sh` - 镜像构建
- **功能**: 从Dockerfile构建MaQueAI镜像
- **包含**: ROS Noetic + 所有依赖库
- **用途**: 创建开发环境

### 🔧 开发工具 (dev/)

#### `run_format.sh` - 代码格式化
- **功能**: 使用clang-format格式化C++代码
- **支持**: 递归处理所有.cpp/.h文件
- **配置**: 使用项目根目录的.clang-format

#### `verify_setup.sh` - 环境验证
- **功能**: 检查开发环境完整性
- **检查项**: ROS、依赖库、工作空间
- **输出**: 详细的环境状态报告

### 🚁 仿真脚本 (simulation/)

#### `start_px4_simulation.sh` - PX4仿真启动
- **功能**: 启动PX4 SITL + Gazebo仿真
- **环境**: 通常在宿主机运行
- **配置**: 支持多种无人机模型和世界

### 🛠️ 实用工具 (utils/)

#### `explain_structure.sh` - 项目结构说明
- **功能**: 生成项目目录结构图
- **输出**: 可视化的文件树
- **用途**: 帮助理解项目组织

## 🔄 典型工作流程

### 1. 首次设置
```bash
# 1. 构建Docker镜像
./scripts/docker/build_docker.sh

# 2. 启动开发容器
./scripts/docker/run_docker_dev.sh

# 3. 在容器内构建系统
./scripts/build/build_all.sh
```

### 2. 日常开发
```bash
# 1. 启动开发容器
./scripts/docker/run_docker_dev.sh

# 2. 修改代码后重新构建
./scripts/build/build_livo.sh

# 3. 运行仿真测试
./simulation/run_simulation.sh
```

### 3. 代码提交前
```bash
# 1. 格式化代码
./scripts/dev/run_format.sh

# 2. 验证环境
./scripts/dev/verify_setup.sh

# 3. 运行完整测试
./simulation/run_simulation.sh --scenario autonomous_mission
```

## ⚠️ 注意事项

1. **路径问题**: 脚本已修复移动到workspace后的路径问题
2. **权限问题**: 确保所有脚本有执行权限 `chmod +x scripts/**/*.sh`
3. **Docker环境**: 构建脚本需要在Docker容器内运行
4. **宿主机仿真**: PX4仿真通常在宿主机运行以获得更好性能

## 🆘 故障排除

### 常见问题

1. **脚本找不到**: 确保在项目根目录运行
2. **权限错误**: 运行 `chmod +x scripts/**/*.sh`
3. **Docker连接失败**: 检查Docker服务是否运行
4. **构建失败**: 查看具体错误信息，通常是依赖问题

### 获取帮助

```bash
# 查看脚本帮助
./scripts/build/build_all.sh --help
./scripts/docker/run_docker_dev.sh --help

# 验证环境状态
./scripts/dev/verify_setup.sh
``` 