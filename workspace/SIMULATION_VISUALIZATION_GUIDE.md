# MaQueAI 仿真与可视化完整指南

## 🎯 概述

本指南提供了MaQueAI系统完整的仿真和可视化解决方案，支持：
- PX4 SITL仿真
- Gazebo 3D可视化
- LIVO定位系统集成
- 多种仿真环境和载具
- GUI和无头模式

## 🚀 快速开始

### 1. 启动支持GUI的容器
```bash
# 启动支持图形界面的Docker容器
./workspace/scripts/docker/run_docker_gui.sh
```

### 2. 编译系统组件
```bash
# 编译PX4（支持Gazebo）
./scripts/build/build_px4.sh

# 编译LIVO
./scripts/build/build_livo.sh
```

### 3. 启动仿真
```bash
# 基础PX4仿真（iris四旋翼）
./scripts/simulation/start_px4_sim.sh

# 或独立Gazebo环境
./scripts/simulation/start_gazebo_sim.sh
```

## 📋 脚本说明

### Docker容器脚本

#### `run_docker_gui.sh` - GUI支持容器
**功能**: 启动支持X11显示的Docker容器
**特点**:
- 自动配置X11权限
- 支持Gazebo、RViz等GUI应用
- 硬件加速支持
- 音频和视频设备访问

**使用方法**:
```bash
./workspace/scripts/docker/run_docker_gui.sh          # 启动GUI容器
./workspace/scripts/docker/run_docker_gui.sh --check  # 检查X11支持
./workspace/scripts/docker/run_docker_gui.sh --help   # 显示帮助
```

### 仿真启动脚本

#### `start_px4_sim.sh` - PX4仿真启动器
**功能**: 智能启动PX4 SITL仿真
**支持载具**:
- `iris` - 基础四旋翼
- `iris_obs_avoid` - 带避障四旋翼
- `typhoon_h480` - 大型六旋翼
- `plane` - 固定翼
- `standard_vtol` - 垂直起降
- `rover` - 地面机器人
- `boat` - 水面载具

**支持环境**:
- `empty` - 空环境
- `warehouse` - 仓库环境
- `windy` - 有风环境
- `sonoma_raceway` - 赛道环境

**使用方法**:
```bash
./scripts/simulation/start_px4_sim.sh                    # 默认iris在空世界
./scripts/simulation/start_px4_sim.sh typhoon_h480      # 六旋翼
./scripts/simulation/start_px4_sim.sh iris warehouse    # 四旋翼在仓库
./scripts/simulation/start_px4_sim.sh plane empty headless  # 固定翼无GUI
```

#### `start_gazebo_sim.sh` - Gazebo独立启动器
**功能**: 启动独立Gazebo环境或与PX4集成
**模式**:
- `standalone` - 独立Gazebo环境
- `px4` - 与PX4集成模式

**使用方法**:
```bash
./scripts/simulation/start_gazebo_sim.sh                      # 独立空世界
./scripts/simulation/start_gazebo_sim.sh warehouse           # 独立仓库环境
./scripts/simulation/start_gazebo_sim.sh empty px4 iris      # PX4集成模式
./scripts/simulation/start_gazebo_sim.sh windy standalone "" false  # 无GUI模式
```

### 编译脚本优化

#### `build_px4.sh` - 增强版PX4编译
**新增功能**:
- 完善的Python环境检测和修复
- 智能依赖安装
- 多层次编译策略（Gazebo -> SITL -> 核心）
- 详细的错误诊断

**编译层次**:
1. **完整编译**: PX4 + Gazebo + 所有插件
2. **基础SITL**: PX4核心仿真（无Gazebo）
3. **最小编译**: 仅核心组件

## 🖥️ 可视化配置

### X11显示配置

#### 本地环境
```bash
# 设置显示环境
export DISPLAY=:0

# 启动GUI容器
./workspace/scripts/docker/run_docker_gui.sh
```

#### 远程SSH环境
```bash
# SSH连接时启用X11转发
ssh -X username@remote-server

# 或在SSH配置中添加
# ~/.ssh/config:
Host remote-server
    ForwardX11 yes
    ForwardX11Trusted yes
```

#### WSL2环境
```bash
# 安装X11服务器（如VcXsrv）
# 设置DISPLAY变量
export DISPLAY=$(ip route list default | awk '{print $3}'):0

# 启动GUI容器
./workspace/scripts/docker/run_docker_gui.sh
```

### Gazebo可视化

#### 界面控制
- **鼠标左键拖拽**: 旋转视角
- **鼠标滚轮**: 缩放
- **鼠标中键拖拽**: 平移视角
- **Ctrl+R**: 重置仿真
- **空格**: 暂停/继续仿真

#### 性能优化
```bash
# 在容器内设置环境变量以提高性能
export GAZEBO_MODEL_DATABASE_URI=""  # 禁用在线模型下载
export GAZEBO_VERBOSE=1              # 启用详细日志
export OGRE_RTT_MODE=Copy            # 优化渲染
```

## 🔌 连接接口

### MAVLink连接
- **UDP端口**: 14540 (QGroundControl自动连接)
- **TCP端口**: 4560 (MAVSDK/自定义应用)
- **本地连接**: `udp://localhost:14540`
- **远程连接**: `udp://0.0.0.0:14540`

### ROS接口
```bash
# 在容器内启动MAVROS
roslaunch mavros px4.launch fcu_url:="tcp://localhost:4560"

# 或使用UDP连接
roslaunch mavros px4.launch fcu_url:="udp://:14540@localhost:14557"
```

## 🧪 仿真场景

### 基础飞行测试
```bash
# 启动基础四旋翼仿真
./scripts/simulation/start_px4_sim.sh iris empty gui

# 在PX4控制台中执行（容器内另开终端）:
commander takeoff    # 起飞
commander land       # 降落
```

### 复杂环境测试
```bash
# 仓库环境导航测试
./scripts/simulation/start_px4_sim.sh iris warehouse gui

# 有风环境稳定性测试
./scripts/simulation/start_px4_sim.sh iris windy gui

# 固定翼长距离飞行
./scripts/simulation/start_px4_sim.sh plane sonoma_raceway gui
```

### LIVO集成测试
```bash
# 1. 启动PX4仿真
./scripts/simulation/start_px4_sim.sh iris warehouse gui

# 2. 在新终端启动LIVO
cd /workspace/src/livo
roslaunch livo livo_px4.launch

# 3. 在新终端运行自主控制脚本
python3 /workspace/simulation/scripts/run_scenario.py basic_flight.yaml
```

## 🐛 常见问题解决

### GUI显示问题
```bash
# 问题: 无法显示Gazebo界面
# 解决方案:
1. 检查DISPLAY环境变量
   echo $DISPLAY
   
2. 检查X11权限
   xhost +local:docker
   
3. 重启GUI容器
   ./workspace/scripts/docker/run_docker_gui.sh
```

### Python模块问题
```bash
# 问题: empy模块未找到
# 解决方案:
1. 重新编译PX4（自动修复）
   ./scripts/build/build_px4.sh
   
2. 手动安装模块
   pip3 install --user empy==3.3.2
```

### Gazebo启动缓慢
```bash
# 解决方案: 禁用在线模型下载
export GAZEBO_MODEL_DATABASE_URI=""

# 或使用离线模式
gazebo --verbose -u world_file.world
```

### 资源不足
```bash
# 问题: 仿真运行缓慢
# 解决方案:
1. 使用无GUI模式
   ./scripts/simulation/start_px4_sim.sh iris empty headless
   
2. 降低仿真实时因子
   # 在Gazebo中设置real_time_factor < 1.0
   
3. 使用轻量级世界
   ./scripts/simulation/start_px4_sim.sh iris empty
```

## 📊 性能监控

### 系统资源监控
```bash
# 在容器内监控资源使用
htop
iostat -x 1
nvidia-smi  # 如果使用GPU加速
```

### 仿真性能指标
```bash
# Gazebo性能统计
gz stats

# PX4性能监控
# 在PX4控制台:
top
perf
```

## 🔧 高级配置

### 自定义仿真环境
1. **添加新世界文件**:
   ```bash
   # 复制到Gazebo世界目录
   cp custom.world /workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/
   ```

2. **添加新载具模型**:
   ```bash
   # 复制到模型目录
   cp -r custom_vehicle /workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
   ```

### 多机仿真
```bash
# 启动多个PX4实例（需要修改端口）
PX4_SIM_MODEL=iris_1 make px4_sitl gazebo_iris__warehouse
PX4_SIM_MODEL=iris_2 make px4_sitl gazebo_iris__warehouse___iris_1
```

## 📝 日志和调试

### 日志位置
- **PX4日志**: `/workspace/px4/build/px4_sitl_default/tmp/rootfs/log/`
- **Gazebo日志**: `~/.gazebo/`
- **LIVO日志**: `/workspace/src/livo/log/`

### 调试工具
```bash
# GDB调试PX4
gdb /workspace/px4/build/px4_sitl_default/bin/px4

# 查看MAVLink消息
mavproxy.py --master=tcp:localhost:4560

# ROS话题监控
rostopic list
rostopic echo /mavros/state
```

## 🎯 下一步

1. **运行完整系统仿真**:
   ```bash
   ./simulation/run_simulation.sh basic_flight.yaml
   ```

2. **开发自定义控制器**:
   - 参考 `/workspace/simulation/scripts/` 中的示例
   - 使用MAVSDK或MAVROS接口

3. **性能评估**:
   - 使用仿真数据分析工具
   - 生成性能报告

## 📞 支持

如遇问题，请检查：
1. 本指南的常见问题部分
2. 容器日志: `docker logs maqueai_gui_container`
3. 脚本帮助: `script_name.sh --help`

---
**MaQueAI Team** - 提供完整的无人机仿真解决方案 