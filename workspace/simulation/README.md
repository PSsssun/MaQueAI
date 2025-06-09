# 🚁 MaQueAI 仿真测试环境

## 📋 概述

这是MaQueAI系统的完整仿真测试环境，提供一键式Docker内仿真测试，包含：

- 🎯 **自动化测试**: 完整的C++系统联合测试
- 🌍 **Gazebo仿真**: 真实物理环境模拟
- 📊 **可视化界面**: RViz实时监控
- 📈 **数据记录**: 自动测试结果分析
- 🛡️ **安全测试**: 多种故障场景验证

## 🏗️ 目录结构

```
simulation/
├── 📄 README.md              # 本文档
├── 📄 run_simulation.sh      # 一键仿真脚本 ⭐
│
├── 📁 scripts/               # 测试脚本
│   ├── 🐍 simulation_manager.py      # 仿真管理器
│   ├── 🐍 scenario_executor.py       # 场景执行器  
│   ├── 🐍 data_collector.py          # 数据收集器
│   ├── 🐍 result_analyzer.py         # 结果分析器
│   └── 🐍 visualization_launcher.py  # 可视化启动器
│
├── 📁 config/                # 配置文件
│   ├── 📄 simulation.yaml            # 仿真参数配置
│   ├── 📄 test_scenarios.yaml        # 测试场景配置
│   ├── 📄 gazebo_settings.yaml       # Gazebo配置
│   └── 📄 px4_params.yaml            # PX4参数配置
│
├── 📁 scenarios/             # 测试场景
│   ├── 📄 basic_flight.yaml          # 基础飞行测试
│   ├── 📄 autonomous_mission.yaml    # 自主任务测试
│   ├── 📄 obstacle_avoidance.yaml    # 避障测试
│   ├── 📄 failure_recovery.yaml      # 故障恢复测试
│   └── 📄 performance_test.yaml      # 性能测试
│
├── 📁 worlds/                # Gazebo世界
│   ├── 🌍 empty_world.world          # 空旷环境
│   ├── 🌍 indoor_complex.world       # 室内复杂环境
│   ├── 🌍 outdoor_terrain.world      # 户外地形
│   └── 🌍 warehouse.world            # 仓库环境
│
├── 📁 models/                # 无人机模型
│   ├── 📁 iris_with_lidar/           # 带激光雷达的Iris
│   ├── 📁 custom_quadrotor/          # 自定义四旋翼
│   └── 📁 sensors/                   # 传感器模型
│
├── 📁 launch/                # ROS启动文件
│   ├── 📄 full_simulation.launch     # 完整仿真启动
│   ├── 📄 gazebo_world.launch        # Gazebo世界启动
│   ├── 📄 px4_sitl.launch            # PX4 SITL启动
│   └── 📄 maqueai_system.launch      # MaQueAI系统启动
│
└── 📁 results/               # 测试结果 (自动生成)
    ├── 📁 logs/                      # 日志文件
    ├── 📁 rosbags/                   # ROS bag录制
    ├── 📁 screenshots/               # 截图
    └── 📁 reports/                   # 测试报告
```

## 🚀 一键运行

在Docker容器内，修改代码后直接运行：

```bash
# 一键启动完整仿真测试
./workspace/simulation/run_simulation.sh

# 可选参数:
./run_simulation.sh --scenario basic_flight    # 指定测试场景
./run_simulation.sh --world indoor_complex     # 指定仿真世界  
./run_simulation.sh --headless                 # 无GUI模式
./run_simulation.sh --record                   # 记录测试数据
```

## 🎮 测试场景

### 基础测试
- ✈️ **起飞降落**: 基本飞行操作
- 🎯 **定点悬停**: 位置控制精度
- 📍 **航点飞行**: 路径跟踪能力

### 进阶测试  
- 🗺️ **SLAM建图**: 实时定位建图
- 🚧 **避障飞行**: 动态障碍物处理
- 🔄 **任务重规划**: 实时路径调整

### 安全测试
- ⚠️ **传感器故障**: 激光雷达失效
- 📡 **通信中断**: MAVROS连接丢失
- 🔋 **低电量处理**: 紧急降落

## 📊 测试指标

系统自动评估以下性能指标：

### 定位精度
- 📍 位置误差 (RMSE)
- 🧭 姿态误差 (角度)
- ⏱️ 延迟时间 (ms)

### 控制性能
- 🎯 跟踪精度 (位置/速度)
- ⚡ 响应速度 (上升时间)
- 📊 稳定性 (超调量)

### 系统性能
- 💻 CPU使用率
- 💾 内存占用
- 📡 通信频率

## 🔧 自定义测试

### 添加新场景
1. 在`scenarios/`创建YAML配置
2. 定义航点、任务、约束条件
3. 运行`./run_simulation.sh --scenario your_scenario`

### 修改仿真环境
1. 编辑`worlds/`中的Gazebo世界文件
2. 调整`config/gazebo_settings.yaml`参数
3. 重新运行仿真

## 🏆 预期结果

成功的仿真测试应该显示：

```
✅ 系统启动成功         (< 30s)
✅ SLAM定位收敛        (< 10s)  
✅ 自主飞行完成        (100% 航点)
✅ 控制精度达标        (位置误差 < 10cm)
✅ 安全降落完成        (无硬着陆)

📊 测试报告已生成: results/reports/test_YYYYMMDD_HHMMSS.html
```

## 🐛 故障排除

常见问题和解决方案请参考 `scripts/troubleshooting.md` 