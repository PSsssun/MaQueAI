#!/bin/bash

echo "👁️  === Docker build real-time monitoring ==="
echo "press Ctrl+C to exit monitoring"
echo ""

# record start time
start_time=$(date +%s)

while true; do
    clear
    current_time=$(date +%s)
    elapsed=$((current_time - start_time))
    elapsed_formatted=$(printf "%02d:%02d:%02d" $((elapsed/3600)) $((elapsed%3600/60)) $((elapsed%60)))
    
    echo "👁️  === Docker build real-time monitoring === $(date '+%H:%M:%S') | monitoring duration: $elapsed_formatted"
    echo ""
    
    # check Docker build process
    if pgrep -f "docker build" > /dev/null; then
        echo "🟢 build status: running"
        
        # show build process details
        echo ""
        echo "📋 build process details:"
        ps aux | grep "docker build" | grep -v grep | head -1 | while read user pid cpu mem vsz rss tty stat start time cmd; do
            echo "   📍 PID: $pid"
            echo "   🔥 CPU usage: $cpu%"
            echo "   💾 memory usage: $mem%"
            echo "   ⏰ running time: $time"
            echo "   👤 user: $user"
        done
        
        # show Docker image build progress
        echo ""
        echo "🏗️  build progress analysis:"
        
        # try to get Docker process details
        docker_pid=$(pgrep -f "docker build" | head -1)
        if [ ! -z "$docker_pid" ]; then
            # get process start time
            proc_start=$(stat -c %Y /proc/$docker_pid 2>/dev/null || echo "0")
            if [ "$proc_start" != "0" ]; then
                build_elapsed=$((current_time - proc_start))
                build_time_formatted=$(printf "%02d:%02d:%02d" $((build_elapsed/3600)) $((build_elapsed%3600/60)) $((build_elapsed%60)))
                echo "   ⏱️  构建已用时间: $build_time_formatted"
            fi
            
            # show process status
            if [ -f "/proc/$docker_pid/status" ]; then
                state=$(grep "State:" /proc/$docker_pid/status 2>/dev/null | awk '{print $2}')
                echo "   📊 进程状态: $state"
            fi
        fi
        
        # show system resource usage
        echo ""
        echo "💻 系统资源使用:"
        
        # CPU usage
        cpu_usage=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)
        echo "   🔥 CPU总使用率: ${cpu_usage}%"
        
        # memory usage
        mem_info=$(free -h | grep "Mem:")
        total_mem=$(echo $mem_info | awk '{print $2}')
        used_mem=$(echo $mem_info | awk '{print $3}')
        echo "   💾 内存使用: $used_mem / $total_mem"
        
        # disk usage
        disk_usage=$(df -h . | tail -1 | awk '{print $5}')
        echo "   💿 磁盘使用率: $disk_usage"
        
        # Docker specific information
        echo ""
        echo "🐳 Docker information:"
        
        # try to get Docker information
        if docker system df &>/dev/null; then
            echo "   📦 Docker storage usage:"
            docker system df | tail -n +2 | while read type total active size reclaimable; do
                echo "      $type: $size (reclaimable: $reclaimable)"
            done
        else
            echo "   ❌ cannot access Docker system information"
        fi
        
        # show what is being downloaded (via network monitoring)
        echo ""
        echo "🌐 network activity:"
        
        # network interface statistics
        rx_bytes=$(cat /proc/net/dev | grep -E "(eth0|wlan0|ens|wlp)" | head -1 | awk '{print $2}' 2>/dev/null || echo "0")
        tx_bytes=$(cat /proc/net/dev | grep -E "(eth0|wlan0|ens|wlp)" | head -1 | awk '{print $10}' 2>/dev/null || echo "0")
        
        if [ "$rx_bytes" != "0" ]; then
            rx_mb=$((rx_bytes / 1024 / 1024))
            tx_mb=$((tx_bytes / 1024 / 1024))
            echo "   📥 total received: ${rx_mb}MB"
            echo "   📤 total sent: ${tx_mb}MB"
        else
            echo "   📡 network monitoring not available"
        fi
        
    else
        echo "🔴 build status: not running"
        echo ""
        echo "💡 possible reasons:"
        echo "   - build not started"
        echo "   - build completed"
        echo "   - build stopped or failed"
        echo ""
        echo "📊 current Docker image:"
        docker images --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedSince}}" | head -4
    fi
    
    echo ""
    echo "════════════════════════════════════════"
    echo "⏰ current time: $(date '+%Y-%m-%d %H:%M:%S')"
    echo "🔄 refresh in 5 seconds (press Ctrl+C to exit)"
    
    sleep 5
done 