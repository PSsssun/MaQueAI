# 🚁 MaQueAI 完整仿真测试系统

## 🎯 系统概述

**MaQueAI Simulation System** 是一个**Docker内一键式**的完整自主飞行测试环境，专为MaQueAI C++高性能系统设计。

### ✨ 核心特性

- 🔄 **一键仿真**: 修改代码后直接运行，无需额外配置
- 🏗️ **自动编译**: 集成C++系统的完整编译流程
- 🎮 **多场景测试**: 从基础飞行到复杂自主任务
- 📊 **实时监控**: 性能指标实时收集和分析
- 📈 **自动报告**: HTML格式的详细测试报告
- 🛡️ **故障测试**: 传感器故障、通信中断等安全测试

## 🏗️ 系统架构

```
workspace/
├── src/                    # MaQueAI C++源码
│   ├── localization/       # 定位模块 (50Hz)
│   ├── planning/           # 规划模块 (10Hz)
│   ├── control/            # 控制模块 (100Hz)
│   ├── interface/          # 接口模块 (20Hz)
│   ├── launch/             # 启动文件
│   └── LIVO/               # SLAM系统
├── px4/                    # PX4飞控固件
└── simulation/             # 仿真测试环境 ⭐
    ├── run_simulation.sh   # 一键仿真脚本 🚀
    ├── scripts/            # 测试脚本
    ├── scenarios/          # 测试场景
    ├── worlds/             # Gazebo世界
    ├── config/             # 配置文件
    ├── launch/             # 启动文件
    └── results/            # 测试结果 (自动生成)
```

## 🚀 一键使用

### 第一步：启动Docker容器
```bash
# 在主机上启动Docker容器
cd MaQueAI
./run_docker.sh
```

### 第二步：运行仿真测试
```bash
# 在容器内直接运行
./workspace/simulation/run_simulation.sh

# 可选参数示例:
./workspace/simulation/run_simulation.sh --scenario autonomous_mission
./workspace/simulation/run_simulation.sh --world indoor_complex --record
./workspace/simulation/run_simulation.sh --headless --duration 600
```

### 第三步：查看结果
```bash
# 测试完成后，结果自动保存在:
ls workspace/simulation/results/test_*_YYYYMMDD_HHMMSS/

# 查看HTML报告:
firefox workspace/simulation/results/test_*/reports/*_report.html
```

## 🎮 测试场景

### 1. 基础飞行测试 (`basic_flight`)
- ✈️ 起飞、悬停、航点飞行、降落
- 🎯 4个简单航点，正方形路径
- ⏱️ 测试时长: 2分钟
- 🏆 适合: 基础功能验证

### 2. 自主任务测试 (`autonomous_mission`)
- 🗺️ 复杂航点任务 + 圆形飞行 + 8字飞行
- 📡 SLAM建图质量检测
- 🔄 动态重规划测试
- ⏱️ 测试时长: 5分钟
- 🏆 适合: 系统集成测试

### 3. 避障测试 (`obstacle_avoidance`) (待实现)
- 🚧 动态障碍物处理
- 🔄 实时路径重规划
- 🛡️ 安全距离保持

### 4. 故障恢复测试 (`failure_recovery`) (待实现)
- ⚠️ 传感器故障模拟
- 📡 通信中断处理
- 🔋 低电量应急降落

## 📊 测试指标

系统自动评估以下关键指标：

### 🎯 飞行性能
- **航点完成率**: 成功到达的航点百分比
- **位置精度**: 平均位置误差 (目标: <0.3m)
- **响应时间**: 控制响应延迟 (目标: <0.5s)
- **飞行稳定性**: 超调量和震荡程度

### ⚡ 系统性能
- **定位频率**: SLAM输出频率 (目标: >45Hz)
- **控制频率**: 控制循环频率 (目标: >95Hz)
- **CPU使用率**: 计算资源占用 (目标: <80%)
- **内存使用率**: 内存资源占用 (目标: <70%)

### 🛡️ 安全指标
- **地理围栏**: 是否超出安全边界
- **速度限制**: 是否超过最大速度
- **姿态角度**: 是否超过最大倾斜角

## 📁 输出结果

每次测试都会生成时间戳命名的结果目录：

```
results/test_basic_flight_20241201_123045/
├── logs/                   # 系统日志
│   ├── ros_logs/           # ROS节点日志
│   └── component_logs/     # 组件详细日志
├── rosbags/                # ROS数据包 (如果启用记录)
│   └── simulation_data.bag
├── reports/                # 分析报告
│   ├── final_report.txt    # 文本报告
│   ├── basic_flight_report.html  # HTML详细报告 ⭐
│   └── basic_flight_analysis.png # 可视化图表
├── data/                   # 原始数据
│   ├── performance_snapshot.json
│   ├── ros_topics_info.json
│   └── system_info.json
├── system/                 # 系统信息
│   └── system_info.json
└── scenario_results.yaml   # 测试结果摘要
```

## 🔧 自定义测试

### 添加新测试场景

1. **创建场景配置**:
```bash
cp scenarios/basic_flight.yaml scenarios/my_test.yaml
# 编辑 my_test.yaml，修改航点和参数
```

2. **运行自定义场景**:
```bash
./run_simulation.sh --scenario my_test
```

### 修改系统参数

编辑配置文件来调整系统行为：
- `config/simulation.yaml` - 仿真环境设置
- `launch/maqueai_system.launch` - ROS节点参数
- `scenarios/*.yaml` - 测试场景配置

### 添加新的Gazebo世界

1. 在 `worlds/` 目录创建新的 `.world` 文件
2. 使用参数 `--world your_world` 启动测试

## 🐛 故障排除

### 常见问题

1. **脚本执行权限错误**
```bash
chmod +x workspace/simulation/run_simulation.sh
chmod +x workspace/simulation/scripts/*.py
```

2. **Docker环境检查失败**
```bash
# 确保在Docker容器内运行
ls /.dockerenv  # 应该存在
```

3. **ROS环境未设置**
```bash
source /opt/ros/noetic/setup.bash
source workspace/devel/setup.bash
```

4. **编译失败**
```bash
# 清理并重新编译
cd workspace
rm -rf build/ devel/
./simulation/run_simulation.sh --no-clean
```

### 调试模式

启用详细输出进行调试：
```bash
./run_simulation.sh --scenario basic_flight --verbose
```

查看特定组件日志：
```bash
# 查看定位模块日志
rostopic echo /localization/status

# 查看控制模块日志  
rostopic echo /control/status

# 查看系统性能
htop
```

## 📈 性能优化建议

### 1. 系统配置优化
```bash
# 设置CPU性能模式
sudo cpupower frequency-set -g performance

# 增加实时优先级
sudo sysctl -w kernel.sched_rt_runtime_us=-1
```

### 2. ROS参数调优
```xml
<!-- 在launch文件中添加 -->
<param name="tcp_nodelay" value="true"/>
<param name="udp_buffer_size" value="65536"/>
```

### 3. 编译优化
```bash
# 使用本机指令集优化
export CMAKE_CXX_FLAGS="-O3 -march=native -mtune=native"
```

## 🎓 进阶用法

### 批量测试
```bash
# 运行多个场景的批量测试
for scenario in basic_flight autonomous_mission; do
    ./run_simulation.sh --scenario $scenario --headless
done
```

### 数据分析
```bash
# 手动运行结果分析
python3 scripts/result_analyzer.py \
    --result_dir results/test_basic_flight_20241201_123045 \
    --scenario basic_flight \
    --generate_report
```

### 性能基准测试
```bash
# 长时间稳定性测试
./run_simulation.sh --scenario autonomous_mission --duration 1800  # 30分钟
```

## 🏆 期望的测试结果

### ✅ 成功标准
- 航点完成率 ≥ 80%
- 位置误差 ≤ 0.5m
- 定位频率 ≥ 45Hz
- 控制频率 ≥ 95Hz
- 零系统崩溃

### 📊 典型性能
在标准配置下，期望看到：
- 🎯 基础测试: 95%+ 成功率
- 🗺️ 自主任务: 85%+ 成功率  
- ⚡ 定位频率: 48-52Hz
- 🎮 控制频率: 98-102Hz
- 💻 CPU使用: 40-60%

## 🔮 未来扩展

计划中的功能改进：
- 🌤️ 天气条件仿真 (风、雨、雾)
- 🏢 更复杂的3D环境 (多层建筑、树林)
- 🤖 多机协同测试
- 🔬 AI算法性能对比
- 📱 Web界面实时监控

---

## 💡 快速开始总结

**第一次使用？只需3步：**

1. `./run_docker.sh` (启动容器)
2. `./workspace/simulation/run_simulation.sh` (运行测试)
3. 打开 `results/test_*/reports/*_report.html` (查看结果)

**就这么简单！** 🎉

这个仿真系统让您可以专注于算法开发，而无需担心测试环境的复杂配置。 