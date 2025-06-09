#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MaQueAI 数据收集器
收集系统日志、性能数据和测试结果
"""

import os
import sys
import argparse
import shutil
import glob
import json
import time
from datetime import datetime
import subprocess

class DataCollector:
    """数据收集器"""
    
    def __init__(self, result_dir, ros_log_dir=None):
        """
        初始化数据收集器
        
        Args:
            result_dir: 结果输出目录
            ros_log_dir: ROS日志目录
        """
        self.result_dir = result_dir
        self.ros_log_dir = ros_log_dir or os.path.expanduser("~/.ros/log")
        
        # 创建目录结构
        self.logs_dir = os.path.join(result_dir, "logs")
        self.data_dir = os.path.join(result_dir, "data")
        self.system_dir = os.path.join(result_dir, "system")
        
        for dir_path in [self.logs_dir, self.data_dir, self.system_dir]:
            os.makedirs(dir_path, exist_ok=True)
            
        print(f"📁 数据收集器已初始化: {result_dir}")
    
    def collect_ros_logs(self):
        """收集ROS日志"""
        print("📝 收集ROS日志...")
        
        try:
            # 获取最新的ROS日志目录
            if os.path.exists(self.ros_log_dir):
                log_dirs = [d for d in os.listdir(self.ros_log_dir) 
                           if os.path.isdir(os.path.join(self.ros_log_dir, d))]
                
                if log_dirs:
                    # 选择最新的日志目录
                    latest_log_dir = max(log_dirs, 
                                       key=lambda d: os.path.getctime(
                                           os.path.join(self.ros_log_dir, d)))
                    
                    src_dir = os.path.join(self.ros_log_dir, latest_log_dir)
                    dst_dir = os.path.join(self.logs_dir, "ros_logs")
                    
                    if os.path.exists(src_dir):
                        shutil.copytree(src_dir, dst_dir, dirs_exist_ok=True)
                        print(f"✅ ROS日志已收集: {dst_dir}")
                    else:
                        print("⚠️  未找到ROS日志")
                else:
                    print("⚠️  ROS日志目录为空")
            else:
                print(f"⚠️  ROS日志目录不存在: {self.ros_log_dir}")
                
        except Exception as e:
            print(f"❌ 收集ROS日志失败: {e}")
    
    def collect_system_info(self):
        """收集系统信息"""
        print("💻 收集系统信息...")
        
        system_info = {
            "timestamp": datetime.now().isoformat(),
            "hostname": self._run_command("hostname").strip(),
            "kernel": self._run_command("uname -r").strip(),
            "os_release": self._read_file("/etc/os-release"),
            "cpu_info": self._get_cpu_info(),
            "memory_info": self._get_memory_info(),
            "disk_info": self._get_disk_info(),
            "ros_info": self._get_ros_info(),
            "docker_info": self._get_docker_info()
        }
        
        # 保存系统信息
        info_file = os.path.join(self.system_dir, "system_info.json")
        with open(info_file, 'w') as f:
            json.dump(system_info, f, indent=2)
        
        print(f"✅ 系统信息已保存: {info_file}")
    
    def collect_performance_data(self):
        """收集性能数据"""
        print("📊 收集性能数据...")
        
        try:
            # CPU使用率
            cpu_usage = self._run_command("top -bn1 | grep 'Cpu(s)' | awk '{print $2}' | cut -d'%' -f1")
            
            # 内存使用率
            mem_info = self._run_command("free -h")
            
            # 磁盘使用率
            disk_info = self._run_command("df -h")
            
            # 网络统计
            net_info = self._run_command("cat /proc/net/dev")
            
            # ROS节点状态
            ros_nodes = self._run_command("rosnode list 2>/dev/null || echo 'ROS not running'")
            
            performance_data = {
                "timestamp": datetime.now().isoformat(),
                "cpu_usage": cpu_usage.strip(),
                "memory_info": mem_info,
                "disk_info": disk_info,
                "network_info": net_info,
                "ros_nodes": ros_nodes.split('\n')
            }
            
            # 保存性能数据
            perf_file = os.path.join(self.data_dir, "performance_snapshot.json")
            with open(perf_file, 'w') as f:
                json.dump(performance_data, f, indent=2)
            
            print(f"✅ 性能数据已保存: {perf_file}")
            
        except Exception as e:
            print(f"❌ 收集性能数据失败: {e}")
    
    def collect_ros_topics_info(self):
        """收集ROS话题信息"""
        print("🔗 收集ROS话题信息...")
        
        try:
            # 获取话题列表
            topics_list = self._run_command("rostopic list 2>/dev/null || echo 'ROS not running'")
            
            topics_info = {}
            if "ROS not running" not in topics_list:
                for topic in topics_list.strip().split('\n'):
                    if topic.strip():
                        try:
                            # 获取话题信息
                            topic_info = self._run_command(f"rostopic info {topic} 2>/dev/null")
                            topic_hz = self._run_command(f"timeout 5 rostopic hz {topic} 2>/dev/null | tail -1")
                            
                            topics_info[topic] = {
                                "info": topic_info,
                                "frequency": topic_hz.strip() if topic_hz.strip() else "unknown"
                            }
                        except:
                            topics_info[topic] = {"info": "error", "frequency": "error"}
            
            # 保存话题信息
            topics_file = os.path.join(self.data_dir, "ros_topics_info.json")
            with open(topics_file, 'w') as f:
                json.dump(topics_info, f, indent=2)
            
            print(f"✅ ROS话题信息已保存: {topics_file}")
            
        except Exception as e:
            print(f"❌ 收集ROS话题信息失败: {e}")
    
    def collect_test_artifacts(self):
        """收集测试产物"""
        print("📦 收集测试产物...")
        
        # 查找rosbag文件
        bag_files = glob.glob(os.path.join(self.result_dir, "**/*.bag"), recursive=True)
        if bag_files:
            print(f"✅ 找到 {len(bag_files)} 个rosbag文件")
            
            # 创建bag信息
            bag_info = {}
            for bag_file in bag_files:
                try:
                    info = self._run_command(f"rosbag info {bag_file}")
                    bag_info[os.path.basename(bag_file)] = {
                        "path": bag_file,
                        "size": os.path.getsize(bag_file),
                        "info": info
                    }
                except:
                    bag_info[os.path.basename(bag_file)] = {
                        "path": bag_file,
                        "size": os.path.getsize(bag_file),
                        "info": "error"
                    }
            
            # 保存bag信息
            bag_info_file = os.path.join(self.data_dir, "rosbag_info.json")
            with open(bag_info_file, 'w') as f:
                json.dump(bag_info, f, indent=2)
        
        # 查找其他测试文件
        test_files = []
        for ext in ['*.yaml', '*.json', '*.csv', '*.txt']:
            test_files.extend(glob.glob(os.path.join(self.result_dir, "**/" + ext), recursive=True))
        
        if test_files:
            print(f"✅ 找到 {len(test_files)} 个测试文件")
    
    def generate_collection_report(self):
        """生成收集报告"""
        print("📋 生成收集报告...")
        
        report = {
            "collection_time": datetime.now().isoformat(),
            "result_directory": self.result_dir,
            "collected_items": {
                "ros_logs": os.path.exists(os.path.join(self.logs_dir, "ros_logs")),
                "system_info": os.path.exists(os.path.join(self.system_dir, "system_info.json")),
                "performance_data": os.path.exists(os.path.join(self.data_dir, "performance_snapshot.json")),
                "ros_topics_info": os.path.exists(os.path.join(self.data_dir, "ros_topics_info.json"))
            },
            "directory_sizes": {}
        }
        
        # 计算目录大小
        for dir_name in ["logs", "data", "system"]:
            dir_path = os.path.join(self.result_dir, dir_name)
            if os.path.exists(dir_path):
                size = self._get_directory_size(dir_path)
                report["directory_sizes"][dir_name] = f"{size / 1024 / 1024:.2f} MB"
        
        # 保存报告
        report_file = os.path.join(self.result_dir, "collection_report.json")
        with open(report_file, 'w') as f:
            json.dump(report, f, indent=2)
        
        print(f"✅ 收集报告已生成: {report_file}")
        return report
    
    def _run_command(self, command):
        """运行系统命令"""
        try:
            result = subprocess.run(command, shell=True, capture_output=True, text=True, timeout=10)
            return result.stdout
        except:
            return "command_failed"
    
    def _read_file(self, file_path):
        """读取文件内容"""
        try:
            with open(file_path, 'r') as f:
                return f.read()
        except:
            return "file_not_found"
    
    def _get_cpu_info(self):
        """获取CPU信息"""
        try:
            cpu_info = self._run_command("lscpu | grep 'Model name' | cut -d: -f2")
            cpu_cores = self._run_command("nproc")
            return {
                "model": cpu_info.strip(),
                "cores": cpu_cores.strip()
            }
        except:
            return {"model": "unknown", "cores": "unknown"}
    
    def _get_memory_info(self):
        """获取内存信息"""
        try:
            mem_total = self._run_command("grep MemTotal /proc/meminfo | awk '{print $2}'")
            mem_available = self._run_command("grep MemAvailable /proc/meminfo | awk '{print $2}'")
            return {
                "total_kb": mem_total.strip(),
                "available_kb": mem_available.strip()
            }
        except:
            return {"total_kb": "unknown", "available_kb": "unknown"}
    
    def _get_disk_info(self):
        """获取磁盘信息"""
        try:
            disk_usage = self._run_command("df -h / | tail -1")
            return disk_usage.strip()
        except:
            return "unknown"
    
    def _get_ros_info(self):
        """获取ROS信息"""
        try:
            ros_distro = os.environ.get("ROS_DISTRO", "unknown")
            ros_master = self._run_command("echo $ROS_MASTER_URI")
            return {
                "distro": ros_distro,
                "master_uri": ros_master.strip()
            }
        except:
            return {"distro": "unknown", "master_uri": "unknown"}
    
    def _get_docker_info(self):
        """获取Docker信息"""
        try:
            if os.path.exists("/.dockerenv"):
                container_id = self._run_command("cat /proc/self/cgroup | head -1 | cut -d/ -f3")
                return {
                    "in_container": True,
                    "container_id": container_id.strip()[:12]
                }
            else:
                return {"in_container": False}
        except:
            return {"in_container": "unknown"}
    
    def _get_directory_size(self, directory):
        """获取目录大小(字节)"""
        total_size = 0
        try:
            for dirpath, dirnames, filenames in os.walk(directory):
                for filename in filenames:
                    filepath = os.path.join(dirpath, filename)
                    if os.path.exists(filepath):
                        total_size += os.path.getsize(filepath)
        except:
            pass
        return total_size


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='MaQueAI 数据收集器')
    parser.add_argument('--result_dir', required=True, help='结果输出目录')
    parser.add_argument('--ros_log_dir', help='ROS日志目录')
    parser.add_argument('--verbose', action='store_true', help='详细输出')
    
    args = parser.parse_args()
    
    try:
        collector = DataCollector(args.result_dir, args.ros_log_dir)
        
        print("🚀 开始数据收集...")
        
        # 执行收集任务
        collector.collect_system_info()
        collector.collect_performance_data()
        collector.collect_ros_logs()
        collector.collect_ros_topics_info()
        collector.collect_test_artifacts()
        
        # 生成报告
        report = collector.generate_collection_report()
        
        print("✅ 数据收集完成")
        print(f"📊 收集报告: {report}")
        
    except Exception as e:
        print(f"❌ 数据收集失败: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main() 