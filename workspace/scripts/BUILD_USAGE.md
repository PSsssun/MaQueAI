# 🔨 MaQueAI 构建脚本使用指南

## 📋 脚本分工明确

### 🗺️ `build_livo.sh` - 只编译LIVO
```bash
./scripts/build/build_livo.sh
```
**功能**：
- ✅ 只编译 FAST-LIVO2 SLAM系统
- ✅ 自动修复Sophus依赖问题  
- ✅ 设置正确的环境变量
- ❌ 不编译其他MaQueAI模块

**适用场景**：
- 只修改了LIVO相关代码
- 快速测试SLAM功能
- 初次安装LIVO系统

---

### 🚁 `build_px4.sh` - 只编译PX4  
```bash
./scripts/build/build_px4.sh
```
**功能**：
- ✅ 只编译 PX4 SITL仿真版本
- ✅ 安装PX4工具链依赖
- ✅ 支持Gazebo仿真
- ❌ 不编译ROS相关代码

**适用场景**：
- 只修改了PX4相关配置
- 准备仿真环境
- 测试飞控功能

---

### 🏗️ `build_all.sh` - 编译完整系统
```bash
./scripts/build/build_all.sh
```
**功能**：
- ✅ 编译整个catkin工作空间
- ✅ 包含所有MaQueAI C++模块
- ✅ 集成MAVROS通信
- ✅ 创建系统启动配置

**适用场景**：
- 首次完整构建
- 修改了多个模块
- 准备系统集成测试
- 发布版本构建

---

## 🎯 推荐的构建流程

### 🆕 首次构建
```bash
# 1. 构建完整系统
./scripts/build/build_all.sh

# 2. 验证构建结果
source /workspace/catkin_ws/devel/setup.bash
rospack find LIVO
```

### 🔧 日常开发  
```bash
# 只修改了LIVO代码
./scripts/build/build_livo.sh

# 只修改了控制代码
catkin_make --pkg control

# 修改了多个模块
./scripts/build/build_all.sh
```

### 🧪 仿真测试
```bash
# 1. 确保LIVO已构建
./scripts/build/build_livo.sh

# 2. 运行仿真
./simulation/run_simulation.sh
```

## 📊 各脚本对比

| 脚本 | 编译内容 | 时间 | 适用场景 |
|------|----------|------|----------|
| `build_livo.sh` | 只LIVO | ⚡ 快 | SLAM开发 |
| `build_px4.sh` | 只PX4 | ⚡ 快 | 飞控测试 |  
| `build_all.sh` | 全部模块 | 🐌 慢 | 系统集成 |

## ⚠️ 常见错误及解决

### 1. 依赖错误
```bash
# 问题：缺少某个包的依赖
# 解决：运行完整构建
./scripts/build/build_all.sh
```

### 2. Sophus错误  
```bash
# 问题：Sophus库找不到
# 解决：使用LIVO脚本自动修复
./scripts/build/build_livo.sh
```

### 3. 消息文件错误
```bash
# 问题：add_message_files() directory not found
# 解决：创建缺失的msg目录或注释相关代码
mkdir -p src/control/msg
```

## 🚀 快速命令参考

```bash
# 只编译特定包
catkin_make --pkg package_name

# 清理构建
rm -rf build/ devel/

# 设置环境
source devel/setup.bash

# 检查包状态
rospack find package_name

# 列出所有包
rospack list
```

## 💡 性能优化建议

### 并行编译
```bash
# 使用所有CPU核心
catkin_make -j$(nproc)

# 限制并行数（避免内存不足）
catkin_make -j4
```

### 增量编译
```bash
# 只编译修改的包
catkin_make --pkg modified_package

# 跳过测试（加速编译）
catkin_make -DCATKIN_ENABLE_TESTING=OFF
```

---

**总结**：选择合适的脚本可以大大提高开发效率！ 🎯 