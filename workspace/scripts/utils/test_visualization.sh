#!/bin/bash

echo "🧪 === MaQueAI可视化功能测试套件 ==="
echo "Author: MaQueAI Team"
echo "Description: 全面测试Docker GUI、PX4仿真、Gazebo可视化等功能"
echo ""

# 测试结果记录
TEST_RESULTS=()
FAILED_TESTS=()

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 测试状态函数
log_test() {
  local test_name="$1"
  local status="$2"
  local details="$3"

  if [ "$status" = "PASS" ]; then
    echo -e "${GREEN}✅ $test_name: PASS${NC}"
    [ -n "$details" ] && echo "   $details"
    TEST_RESULTS+=("PASS: $test_name")
  elif [ "$status" = "FAIL" ]; then
    echo -e "${RED}❌ $test_name: FAIL${NC}"
    [ -n "$details" ] && echo "   $details"
    TEST_RESULTS+=("FAIL: $test_name")
    FAILED_TESTS+=("$test_name")
  elif [ "$status" = "WARN" ]; then
    echo -e "${YELLOW}⚠️ $test_name: WARNING${NC}"
    [ -n "$details" ] && echo "   $details"
    TEST_RESULTS+=("WARN: $test_name")
  else
    echo -e "${BLUE}ℹ️ $test_name: INFO${NC}"
    [ -n "$details" ] && echo "   $details"
  fi
}

# 测试1: 基础环境检查
test_basic_environment() {
  echo ""
  echo "🔍 测试1: 基础环境检查"
  echo "================================"

  # 检查Docker环境
  if command -v docker >/dev/null 2>&1; then
    if docker info >/dev/null 2>&1; then
      local docker_version=$(docker --version | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+')
      log_test "Docker环境" "PASS" "版本: $docker_version"
    else
      log_test "Docker环境" "FAIL" "Docker守护进程未运行"
    fi
  else
    log_test "Docker环境" "FAIL" "Docker未安装"
  fi

  # 检查项目结构
  if [ -d "/workspace/px4" ] && [ -d "/workspace/livo" ]; then
    log_test "项目结构" "PASS" "PX4和LIVO目录存在"
  else
    log_test "项目结构" "FAIL" "缺少必要的项目目录"
  fi

  # 检查脚本权限
  local script_count=0
  local executable_count=0
  for script in /workspace/scripts/*/*.sh; do
    if [ -f "$script" ]; then
      ((script_count++))
      if [ -x "$script" ]; then
        ((executable_count++))
      fi
    fi
  done

  if [ $executable_count -eq $script_count ] && [ $script_count -gt 0 ]; then
    log_test "脚本权限" "PASS" "$executable_count/$script_count 脚本有执行权限"
  else
    log_test "脚本权限" "WARN" "$executable_count/$script_count 脚本有执行权限"
  fi
}

# 测试2: GUI支持检查
test_gui_support() {
  echo ""
  echo "🖥️ 测试2: GUI支持检查"
  echo "================================"

  # 检查DISPLAY变量
  if [ -n "$DISPLAY" ]; then
    log_test "DISPLAY变量" "PASS" "DISPLAY=$DISPLAY"
  else
    log_test "DISPLAY变量" "FAIL" "未设置DISPLAY环境变量"
  fi

  # 检查X11工具
  if command -v xset >/dev/null 2>&1; then
    if xset q >/dev/null 2>&1; then
      log_test "X11连接" "PASS" "X11服务器连接正常"
    else
      log_test "X11连接" "FAIL" "无法连接到X11服务器"
    fi
  else
    log_test "X11工具" "WARN" "xset命令未找到"
  fi

  # 检查Qt支持
  if [ -n "$QT_QPA_PLATFORM" ]; then
    log_test "Qt平台" "PASS" "QT_QPA_PLATFORM=$QT_QPA_PLATFORM"
  else
    log_test "Qt平台" "WARN" "QT_QPA_PLATFORM未设置"
  fi

  # 检查GPU设备
  if [ -d "/dev/dri" ]; then
    local gpu_devices=$(ls -1 /dev/dri/ | wc -l)
    log_test "GPU设备" "PASS" "检测到 $gpu_devices 个GPU设备"
  else
    log_test "GPU设备" "WARN" "未检测到GPU设备，将使用软件渲染"
  fi
}

# 测试3: Python环境和依赖
test_python_environment() {
  echo ""
  echo "🐍 测试3: Python环境检查"
  echo "================================"

  # Python版本
  local python_version=$(python3 --version 2>&1 | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+')
  if [ -n "$python_version" ]; then
    log_test "Python版本" "PASS" "Python $python_version"
  else
    log_test "Python版本" "FAIL" "Python3未安装或版本获取失败"
  fi

  # 关键Python包
  local packages=("empy" "kconfiglib" "jinja2" "pyserial" "numpy")
  local pass_count=0

  for package in "${packages[@]}"; do
    if python3 -c "import $package" >/dev/null 2>&1; then
      local version=$(python3 -c "import $package; print(getattr($package, '__version__', 'unknown'))" 2>/dev/null)
      log_test "Python包: $package" "PASS" "版本: $version"
      ((pass_count++))
    else
      log_test "Python包: $package" "FAIL" "包未安装或导入失败"
    fi
  done

  if [ $pass_count -eq ${#packages[@]} ]; then
    log_test "Python依赖总体" "PASS" "$pass_count/${#packages[@]} 包正常"
  else
    log_test "Python依赖总体" "FAIL" "$pass_count/${#packages[@]} 包正常"
  fi
}

# 测试4: PX4编译状态
test_px4_build() {
  echo ""
  echo "🚁 测试4: PX4编译状态检查"
  echo "================================"

  # PX4目录检查
  if [ -d "/workspace/px4" ]; then
    log_test "PX4源码" "PASS" "目录存在"

    # 编译文件检查
    if [ -f "/workspace/px4/build/px4_sitl_default/bin/px4" ]; then
      local px4_size=$(stat -c%s "/workspace/px4/build/px4_sitl_default/bin/px4" 2>/dev/null || echo "0")
      if [ "$px4_size" -gt 1000000 ]; then # 至少1MB
        log_test "PX4二进制" "PASS" "大小: $((px4_size / 1024 / 1024))MB"
      else
        log_test "PX4二进制" "FAIL" "文件太小或损坏"
      fi
    else
      log_test "PX4二进制" "FAIL" "未找到编译后的二进制文件"
    fi

    # Gazebo插件检查
    if [ -d "/workspace/px4/build/px4_sitl_default/build_gazebo-classic" ]; then
      local plugin_count=$(find "/workspace/px4/build/px4_sitl_default/build_gazebo-classic" -name "*.so" | wc -l)
      if [ "$plugin_count" -gt 0 ]; then
        log_test "Gazebo插件" "PASS" "找到 $plugin_count 个插件"
      else
        log_test "Gazebo插件" "FAIL" "未找到Gazebo插件"
      fi
    else
      log_test "Gazebo插件" "FAIL" "Gazebo插件目录不存在"
    fi
  else
    log_test "PX4源码" "FAIL" "PX4目录不存在"
  fi
}

# 测试5: Gazebo环境
test_gazebo_environment() {
  echo ""
  echo "🌍 测试5: Gazebo环境检查"
  echo "================================"

  # Gazebo安装检查
  if command -v gazebo >/dev/null 2>&1; then
    local gazebo_version=$(gazebo --version 2>&1 | head -n1 | grep -o '[0-9]\+\.[0-9]\+' || echo "unknown")
    log_test "Gazebo安装" "PASS" "版本: $gazebo_version"
  else
    log_test "Gazebo安装" "FAIL" "Gazebo未安装或不在PATH中"
    return
  fi

  # Gazebo环境变量
  if [ -n "$GAZEBO_MODEL_PATH" ]; then
    log_test "Gazebo模型路径" "PASS" "已设置"
  else
    log_test "Gazebo模型路径" "WARN" "未设置GAZEBO_MODEL_PATH"
  fi

  # 检查PX4模型
  local model_path="/workspace/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
  if [ -d "$model_path" ]; then
    local model_count=$(find "$model_path" -name "*.sdf" | wc -l)
    if [ "$model_count" -gt 0 ]; then
      log_test "PX4模型文件" "PASS" "找到 $model_count 个模型"
    else
      log_test "PX4模型文件" "WARN" "未找到模型文件"
    fi
  else
    log_test "PX4模型文件" "FAIL" "模型目录不存在"
  fi
}

# 测试6: 快速仿真测试
test_quick_simulation() {
  echo ""
  echo "⚡ 测试6: 快速仿真测试"
  echo "================================"

  if [ ! -f "/workspace/px4/build/px4_sitl_default/bin/px4" ]; then
    log_test "仿真测试" "SKIP" "PX4未编译，跳过仿真测试"
    return
  fi

  echo "ℹ️ 启动5秒快速仿真测试..."

  # 设置环境变量
  export HEADLESS=1
  export GAZEBO_HEADLESS=1
  export QT_QPA_PLATFORM=offscreen

  cd /workspace/px4 || {
    log_test "仿真测试" "FAIL" "无法进入PX4目录"
    return
  }

  # 启动快速仿真
  timeout 10s make px4_sitl none_iris >/dev/null 2>&1 &
  local sim_pid=$!

  sleep 5

  if ps -p $sim_pid >/dev/null 2>&1; then
    log_test "仿真启动" "PASS" "仿真成功启动"
    kill $sim_pid >/dev/null 2>&1
    wait $sim_pid >/dev/null 2>&1
  else
    log_test "仿真启动" "FAIL" "仿真启动失败"
  fi
}

# 测试7: 网络连接测试
test_network_connectivity() {
  echo ""
  echo "🌐 测试7: 网络连接测试"
  echo "================================"

  # MAVLink端口测试
  local ports=(14540 14550 4560)
  local available_ports=0

  for port in "${ports[@]}"; do
    if ! netstat -ln 2>/dev/null | grep -q ":$port "; then
      ((available_ports++))
    fi
  done

  if [ $available_ports -eq ${#ports[@]} ]; then
    log_test "MAVLink端口" "PASS" "所有端口可用"
  else
    log_test "MAVLink端口" "WARN" "$available_ports/${#ports[@]} 端口可用"
  fi

  # 检查localhost连接
  if ping -c 1 localhost >/dev/null 2>&1; then
    log_test "本地网络" "PASS" "localhost连接正常"
  else
    log_test "本地网络" "FAIL" "localhost连接失败"
  fi
}

# 显示测试总结
show_test_summary() {
  echo ""
  echo "📊 === 测试总结 ==="
  echo "================================"

  local total_tests=${#TEST_RESULTS[@]}
  local passed_tests=$(printf '%s\n' "${TEST_RESULTS[@]}" | grep -c "^PASS:")
  local failed_tests=$(printf '%s\n' "${TEST_RESULTS[@]}" | grep -c "^FAIL:")
  local warning_tests=$(printf '%s\n' "${TEST_RESULTS[@]}" | grep -c "^WARN:")

  echo "📈 测试统计:"
  echo -e "   总计: $total_tests"
  echo -e "   ${GREEN}通过: $passed_tests${NC}"
  echo -e "   ${RED}失败: $failed_tests${NC}"
  echo -e "   ${YELLOW}警告: $warning_tests${NC}"
  echo ""

  if [ $failed_tests -gt 0 ]; then
    echo -e "${RED}❌ 失败的测试:${NC}"
    for test in "${FAILED_TESTS[@]}"; do
      echo "   - $test"
    done
    echo ""
  fi

  # 给出建议
  echo "💡 建议和解决方案:"

  if [ $failed_tests -eq 0 ]; then
    echo -e "   ${GREEN}🎉 所有关键测试通过！可以开始使用MaQueAI系统${NC}"
    echo ""
    echo "🚀 快速启动命令:"
    echo "   GUI模式: ./scripts/docker/run_docker_gui.sh"
    echo "   仿真测试: ./scripts/simulation/start_px4_sim.sh --gui"
    echo "   Gazebo环境: ./scripts/simulation/start_gazebo_sim.sh"
  else
    echo "   🔧 请根据失败的测试项目进行修复："

    if printf '%s\n' "${FAILED_TESTS[@]}" | grep -q "Docker环境"; then
      echo "   - 启动Docker: sudo systemctl start docker"
    fi

    if printf '%s\n' "${FAILED_TESTS[@]}" | grep -q "X11连接"; then
      echo "   - 设置X11权限: xhost +local:docker"
      echo "   - 检查DISPLAY: echo \$DISPLAY"
    fi

    if printf '%s\n' "${FAILED_TESTS[@]}" | grep -q "PX4"; then
      echo "   - 编译PX4: ./scripts/build/build_px4.sh"
    fi

    if printf '%s\n' "${FAILED_TESTS[@]}" | grep -q "Python"; then
      echo "   - 修复Python环境: pip3 install empy==3.3.2 kconfiglib jinja2"
    fi
  fi

  echo ""
  echo "📖 详细文档: /workspace/docs/"
  echo "🐛 问题报告: https://github.com/your-repo/issues"
}

# 主执行函数
main() {
  echo "🔍 开始全面测试..."
  echo "预计时间: 30-60秒"
  echo ""

  # 执行所有测试
  test_basic_environment
  test_gui_support
  test_python_environment
  test_px4_build
  test_gazebo_environment
  test_quick_simulation
  test_network_connectivity

  # 显示总结
  show_test_summary
}

# 参数处理
case "${1:-}" in
  --help | -h)
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --help, -h     显示此帮助信息"
    echo "  --quick        仅运行快速测试"
    echo "  --full         运行完整测试套件 (默认)"
    echo ""
    echo "功能:"
    echo "  - 检查Docker和GUI环境"
    echo "  - 验证Python依赖和版本"
    echo "  - 测试PX4编译状态"
    echo "  - 检查Gazebo仿真环境"
    echo "  - 执行快速仿真测试"
    echo "  - 网络连接检查"
    exit 0
    ;;
  --quick)
    test_basic_environment
    test_gui_support
    show_test_summary
    ;;
  *)
    main "$@"
    ;;
esac
