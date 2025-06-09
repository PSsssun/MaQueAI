#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MaQueAI 测试场景执行器
执行自动化测试场景，监控系统性能
"""

import rospy
import yaml
import time
import argparse
import os
import sys
from threading import Thread, Event
import numpy as np

# ROS消息类型
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from nav_msgs.msg import Odometry, Path
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class ScenarioExecutor:
    """测试场景执行器"""
    
    def __init__(self, scenario_file, result_dir, duration=300):
        """
        初始化场景执行器
        
        Args:
            scenario_file: 场景配置文件路径
            result_dir: 结果输出目录
            duration: 测试持续时间(秒)
        """
        self.scenario_file = scenario_file
        self.result_dir = result_dir
        self.duration = duration
        self.start_time = None
        self.stop_event = Event()
        
        # 加载场景配置
        self.load_scenario()
        
        # 初始化ROS
        rospy.init_node('scenario_executor', anonymous=True)
        
        # 状态变量
        self.current_pose = None
        self.current_velocity = None
        self.mavros_state = None
        self.system_ready = False
        self.test_results = {
            'scenario': self.scenario['name'],
            'start_time': None,
            'end_time': None,
            'duration': duration,
            'success': False,
            'waypoints_completed': 0,
            'total_waypoints': 0,
            'errors': [],
            'performance_metrics': {}
        }
        
        # 设置ROS订阅器和发布器
        self.setup_ros_interface()
        
        # 性能监控
        self.performance_monitor = PerformanceMonitor()
        
        rospy.loginfo(f"📋 场景执行器已初始化: {self.scenario['name']}")
    
    def load_scenario(self):
        """加载场景配置文件"""
        try:
            with open(self.scenario_file, 'r', encoding='utf-8') as f:
                self.scenario = yaml.safe_load(f)
            rospy.loginfo(f"✅ 场景配置已加载: {self.scenario['name']}")
        except Exception as e:
            rospy.logerr(f"❌ 场景配置加载失败: {e}")
            sys.exit(1)
    
    def setup_ros_interface(self):
        """设置ROS接口"""
        # 订阅器
        self.pose_sub = rospy.Subscriber('/localization/current_pose', PoseStamped, self.pose_callback)
        self.velocity_sub = rospy.Subscriber('/localization/filtered_odom', Odometry, self.velocity_callback)
        self.mavros_state_sub = rospy.Subscriber('/mavros/state', State, self.mavros_state_callback)
        
        # 发布器
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.cmd_pub = rospy.Publisher('/simulation/command', String, queue_size=10)
        
        # 服务客户端
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        
        rospy.loginfo("🔗 ROS接口已设置完成")
    
    def pose_callback(self, msg):
        """位置回调函数"""
        self.current_pose = msg
        self.performance_monitor.update_pose(msg)
    
    def velocity_callback(self, msg):
        """速度回调函数"""
        self.current_velocity = msg
        self.performance_monitor.update_velocity(msg)
    
    def mavros_state_callback(self, msg):
        """MAVROS状态回调函数"""
        self.mavros_state = msg
        if msg.connected and msg.mode == "OFFBOARD" and msg.armed:
            self.system_ready = True
    
    def wait_for_system_ready(self, timeout=30):
        """等待系统准备就绪"""
        rospy.loginfo("⏳ 等待系统准备就绪...")
        
        start_time = time.time()
        while not rospy.is_shutdown() and time.time() - start_time < timeout:
            if (self.current_pose is not None and 
                self.mavros_state is not None and 
                self.mavros_state.connected):
                rospy.loginfo("✅ 系统已准备就绪")
                return True
            
            rospy.sleep(0.5)
        
        rospy.logerr("❌ 系统准备超时")
        return False
    
    def arm_and_takeoff(self, altitude=2.0):
        """解锁并起飞"""
        rospy.loginfo(f"🚁 执行解锁和起飞到 {altitude}m...")
        
        try:
            # 设置OFFBOARD模式
            mode_resp = self.set_mode_client(custom_mode="OFFBOARD")
            if mode_resp.mode_sent:
                rospy.loginfo("✅ OFFBOARD模式设置成功")
            else:
                raise Exception("OFFBOARD模式设置失败")
            
            rospy.sleep(1)
            
            # 解锁
            arm_resp = self.arming_client(True)
            if arm_resp.success:
                rospy.loginfo("✅ 飞行器解锁成功")
            else:
                raise Exception("飞行器解锁失败")
            
            rospy.sleep(1)
            
            # 起飞
            takeoff_resp = self.takeoff_client(altitude=altitude)
            if takeoff_resp.success:
                rospy.loginfo(f"✅ 起飞到 {altitude}m 成功")
                return True
            else:
                raise Exception("起飞失败")
                
        except Exception as e:
            rospy.logerr(f"❌ 解锁起飞失败: {e}")
            self.test_results['errors'].append(f"解锁起飞失败: {e}")
            return False
    
    def execute_waypoint_mission(self):
        """执行航点任务"""
        waypoints = self.scenario.get('waypoints', [])
        self.test_results['total_waypoints'] = len(waypoints)
        
        rospy.loginfo(f"🗺️  开始执行航点任务，共 {len(waypoints)} 个航点")
        
        for i, waypoint in enumerate(waypoints):
            if self.stop_event.is_set():
                break
                
            rospy.loginfo(f"📍 飞向航点 {i+1}/{len(waypoints)}: [{waypoint['x']:.1f}, {waypoint['y']:.1f}, {waypoint['z']:.1f}]")
            
            # 发送目标位置
            target_pose = PoseStamped()
            target_pose.header.stamp = rospy.Time.now()
            target_pose.header.frame_id = "map"
            target_pose.pose.position.x = waypoint['x']
            target_pose.pose.position.y = waypoint['y']
            target_pose.pose.position.z = waypoint['z']
            target_pose.pose.orientation.w = 1.0
            
            # 等待到达航点
            start_time = time.time()
            timeout = waypoint.get('timeout', 30)
            tolerance = waypoint.get('tolerance', 0.5)
            
            while not rospy.is_shutdown() and time.time() - start_time < timeout:
                if self.stop_event.is_set():
                    break
                    
                # 持续发送目标位置
                self.setpoint_pub.publish(target_pose)
                
                # 检查是否到达
                if self.current_pose:
                    distance = self.calculate_distance(
                        [self.current_pose.pose.position.x, 
                         self.current_pose.pose.position.y, 
                         self.current_pose.pose.position.z],
                        [waypoint['x'], waypoint['y'], waypoint['z']]
                    )
                    
                    if distance < tolerance:
                        rospy.loginfo(f"✅ 到达航点 {i+1}，距离误差: {distance:.2f}m")
                        self.test_results['waypoints_completed'] += 1
                        break
                
                rospy.sleep(0.1)
            else:
                rospy.logwarn(f"⚠️  航点 {i+1} 超时，距离误差: {distance:.2f}m")
                self.test_results['errors'].append(f"航点{i+1}超时")
            
            # 航点间等待时间
            wait_time = waypoint.get('wait_time', 2)
            rospy.sleep(wait_time)
    
    def execute_landing(self):
        """执行降落"""
        rospy.loginfo("🛬 开始降落...")
        
        # 发送降落指令
        try:
            # 切换到AUTO.LAND模式
            mode_resp = self.set_mode_client(custom_mode="AUTO.LAND")
            if mode_resp.mode_sent:
                rospy.loginfo("✅ 降落模式设置成功")
            else:
                rospy.logwarn("⚠️  降落模式设置失败，尝试手动降落")
                self.manual_landing()
                
        except Exception as e:
            rospy.logerr(f"❌ 降落失败: {e}")
            self.test_results['errors'].append(f"降落失败: {e}")
    
    def manual_landing(self):
        """手动降落"""
        if not self.current_pose:
            return
            
        # 逐步降低高度
        current_z = self.current_pose.pose.position.z
        while current_z > 0.2 and not rospy.is_shutdown():
            target_pose = PoseStamped()
            target_pose.header.stamp = rospy.Time.now()
            target_pose.header.frame_id = "map"
            target_pose.pose.position.x = self.current_pose.pose.position.x
            target_pose.pose.position.y = self.current_pose.pose.position.y
            target_pose.pose.position.z = max(0.1, current_z - 0.1)
            target_pose.pose.orientation.w = 1.0
            
            self.setpoint_pub.publish(target_pose)
            current_z -= 0.1
            rospy.sleep(0.5)
    
    def calculate_distance(self, pos1, pos2):
        """计算两点间距离"""
        return np.sqrt(sum([(a - b) ** 2 for a, b in zip(pos1, pos2)]))
    
    def run_scenario(self):
        """运行测试场景"""
        rospy.loginfo(f"🚀 开始执行测试场景: {self.scenario['name']}")
        
        self.test_results['start_time'] = time.time()
        self.start_time = time.time()
        
        try:
            # 1. 等待系统准备
            if not self.wait_for_system_ready():
                self.test_results['errors'].append("系统准备超时")
                return False
            
            # 2. 解锁起飞
            takeoff_altitude = self.scenario.get('takeoff_altitude', 2.0)
            if not self.arm_and_takeoff(takeoff_altitude):
                return False
            
            # 等待起飞完成
            rospy.sleep(5)
            
            # 3. 执行航点任务
            if 'waypoints' in self.scenario:
                self.execute_waypoint_mission()
            
            # 4. 执行其他任务
            if 'custom_tasks' in self.scenario:
                self.execute_custom_tasks()
            
            # 5. 降落
            self.execute_landing()
            
            # 6. 等待降落完成
            rospy.sleep(10)
            
            # 计算成功率
            if self.test_results['total_waypoints'] > 0:
                completion_rate = self.test_results['waypoints_completed'] / self.test_results['total_waypoints']
                if completion_rate >= 0.8 and len(self.test_results['errors']) == 0:
                    self.test_results['success'] = True
                    rospy.loginfo("🏆 测试场景执行成功！")
                else:
                    rospy.logwarn(f"⚠️  测试完成但未达到成功标准 (完成率: {completion_rate:.1%})")
            
        except Exception as e:
            rospy.logerr(f"❌ 场景执行异常: {e}")
            self.test_results['errors'].append(f"执行异常: {e}")
        
        finally:
            self.test_results['end_time'] = time.time()
            self.test_results['actual_duration'] = time.time() - self.start_time
            
            # 收集性能指标
            self.test_results['performance_metrics'] = self.performance_monitor.get_metrics()
            
            # 保存结果
            self.save_results()
            
            return self.test_results['success']
    
    def execute_custom_tasks(self):
        """执行自定义任务"""
        custom_tasks = self.scenario.get('custom_tasks', [])
        
        for task in custom_tasks:
            task_type = task.get('type', 'unknown')
            rospy.loginfo(f"🔧 执行自定义任务: {task_type}")
            
            if task_type == 'hover':
                # 悬停任务
                duration = task.get('duration', 5)
                rospy.loginfo(f"🚁 悬停 {duration} 秒")
                rospy.sleep(duration)
                
            elif task_type == 'circle':
                # 圆形飞行
                self.execute_circle_flight(task)
                
            elif task_type == 'figure8':
                # 8字飞行
                self.execute_figure8_flight(task)
    
    def execute_circle_flight(self, task):
        """执行圆形飞行"""
        center = task.get('center', [0, 0, 2])
        radius = task.get('radius', 2.0)
        speed = task.get('speed', 1.0)  # rad/s
        duration = task.get('duration', 10)
        
        rospy.loginfo(f"🔄 执行圆形飞行: 半径={radius}m, 速度={speed}rad/s")
        
        start_time = time.time()
        while time.time() - start_time < duration and not rospy.is_shutdown():
            t = time.time() - start_time
            angle = speed * t
            
            target_pose = PoseStamped()
            target_pose.header.stamp = rospy.Time.now()
            target_pose.header.frame_id = "map"
            target_pose.pose.position.x = center[0] + radius * np.cos(angle)
            target_pose.pose.position.y = center[1] + radius * np.sin(angle)
            target_pose.pose.position.z = center[2]
            target_pose.pose.orientation.w = 1.0
            
            self.setpoint_pub.publish(target_pose)
            rospy.sleep(0.1)
    
    def save_results(self):
        """保存测试结果"""
        result_file = os.path.join(self.result_dir, 'scenario_results.yaml')
        
        try:
            with open(result_file, 'w', encoding='utf-8') as f:
                yaml.dump(self.test_results, f, default_flow_style=False, allow_unicode=True)
            
            rospy.loginfo(f"📄 测试结果已保存: {result_file}")
            
        except Exception as e:
            rospy.logerr(f"❌ 结果保存失败: {e}")


class PerformanceMonitor:
    """性能监控器"""
    
    def __init__(self):
        self.pose_data = []
        self.velocity_data = []
        self.start_time = time.time()
    
    def update_pose(self, pose_msg):
        """更新位置数据"""
        self.pose_data.append({
            'timestamp': time.time() - self.start_time,
            'x': pose_msg.pose.position.x,
            'y': pose_msg.pose.position.y,
            'z': pose_msg.pose.position.z
        })
    
    def update_velocity(self, odom_msg):
        """更新速度数据"""
        self.velocity_data.append({
            'timestamp': time.time() - self.start_time,
            'vx': odom_msg.twist.twist.linear.x,
            'vy': odom_msg.twist.twist.linear.y,
            'vz': odom_msg.twist.twist.linear.z
        })
    
    def get_metrics(self):
        """计算性能指标"""
        metrics = {
            'total_samples': len(self.pose_data),
            'avg_frequency': len(self.pose_data) / (time.time() - self.start_time) if self.pose_data else 0,
            'max_velocity': 0,
            'avg_velocity': 0,
            'position_accuracy': 0
        }
        
        if self.velocity_data:
            velocities = [np.sqrt(v['vx']**2 + v['vy']**2 + v['vz']**2) for v in self.velocity_data]
            metrics['max_velocity'] = max(velocities)
            metrics['avg_velocity'] = np.mean(velocities)
        
        return metrics


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='MaQueAI 测试场景执行器')
    parser.add_argument('--scenario', required=True, help='场景配置文件路径')
    parser.add_argument('--duration', type=int, default=300, help='测试持续时间(秒)')
    parser.add_argument('--result_dir', required=True, help='结果输出目录')
    parser.add_argument('--verbose', action='store_true', help='详细输出')
    
    args = parser.parse_args()
    
    if args.verbose:
        rospy.set_param('/rospy/logger_level', 'DEBUG')
    
    try:
        executor = ScenarioExecutor(args.scenario, args.result_dir, args.duration)
        success = executor.run_scenario()
        
        if success:
            print("✅ 测试场景执行成功")
            sys.exit(0)
        else:
            print("❌ 测试场景执行失败")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n⏹️  用户中断测试")
        sys.exit(2)
    except Exception as e:
        print(f"❌ 执行器异常: {e}")
        sys.exit(3)


if __name__ == '__main__':
    main() 