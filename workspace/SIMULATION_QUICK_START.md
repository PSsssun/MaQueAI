# 🚁 PX4 Gazebo仿真快速启动指南

## ✅ 仿真环境已就绪！

你的PX4 Gazebo仿真环境已经编译成功并可以正常运行。

## 🚀 启动方式

### 1. Headless模式（推荐，无GUI）

```bash
# 进入容器
docker exec -it 6652c995fd0d bash

# 启动PX4仿真（无GUI，性能最佳）
cd /workspace/px4
export HEADLESS=1
export GAZEBO_HEADLESS=1
export QT_QPA_PLATFORM=offscreen
make px4_sitl gazebo_iris
```

### 2. 使用便捷脚本

```bash
# 进入容器
docker exec -it 6652c995fd0d bash

# 使用我们的启动脚本
cd /workspace
./scripts/simulation/start_px4_sim.sh --headless --vehicle iris --world empty
```

### 3. 不同载具类型

```bash
# 四旋翼（默认）
make px4_sitl gazebo_iris

# 固定翼
make px4_sitl gazebo_plane

# 垂直起降
make px4_sitl gazebo_standard_vtol

# 六旋翼
make px4_sitl gazebo_typhoon_h480
```

### 4. 不同环境

```bash
# 空环境（默认）
make px4_sitl gazebo_iris

# 仓库环境
make px4_sitl gazebo_iris__warehouse

# 有风环境
make px4_sitl gazebo_iris__windy
```

## 📡 连接信息

仿真启动后，你可以通过以下方式连接：

- **MAVLink UDP**: `localhost:14540`
- **MAVLink TCP**: `localhost:4560`
- **QGroundControl**: 自动连接到 `localhost:14540`

## 🎮 基本控制命令

在PX4控制台（pxh>）中：

```bash
# 解锁
commander arm

# 起飞
commander takeoff

# 降落
commander land

# 切换模式
commander mode offboard
commander mode manual
```

## 🔧 故障排除

### 如果提示"PX4 server already running"：

```bash
# 清理进程
pkill -f px4
pkill -f gazebo
sleep 2

# 重新启动
make px4_sitl gazebo_iris
```

### 如果需要重新编译：

```bash
cd /workspace/px4
make clean
make px4_sitl gazebo
```

## 📊 性能监控

```bash
# 查看仿真状态
ps aux | grep px4
ps aux | grep gazebo

# 查看端口占用
netstat -tlnp | grep 14540
netstat -tlnp | grep 4560
```

## 🎯 下一步

1. **连接QGroundControl**: 在主机上安装QGroundControl，它会自动连接到仿真
2. **编写控制脚本**: 使用MAVSDK或pymavlink编写自动控制程序
3. **集成LIVO**: 将LIVO定位系统与PX4仿真结合

## 💡 提示

- Headless模式性能最佳，适合算法开发和测试
- 如果需要可视化，可以考虑使用QGroundControl的3D视图
- 仿真数据会保存在 `/workspace/px4/build/px4_sitl_default/tmp/rootfs/log/`

---

🎉 **恭喜！你的PX4 Gazebo仿真环境已经完全可用！** 