#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
轨迹规划器

功能: 基于定位信息生成飞行轨迹，支持探索、目标导航等多种模式
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import String, Int32
import tf2_ros
from enum import Enum

class PlanningMode(Enum):
    IDLE = 0
    EXPLORATION = 1
    WAYPOINT = 2
    TARGET = 3

class TrajectoryPlanner:
    def __init__(self):
        rospy.init_node('trajectory_planner', anonymous=True)
        
        # 参数配置
        self.altitude = rospy.get_param('~altitude', 2.0)
        self.exploration_radius = rospy.get_param('~exploration_radius', 5.0)
        self.waypoint_spacing = rospy.get_param('~waypoint_spacing', 1.0)
        self.plan_rate = rospy.get_param('~plan_rate', 1.0)  # 规划频率
        
        # 状态变量
        self.current_pose = None
        self.current_mode = PlanningMode.IDLE
        self.current_path = None
        self.current_waypoint_index = 0
        self.target_pose = None
        
        # 探索航点
        self.exploration_waypoints = self.generate_exploration_waypoints()
        
        # 发布器
        self.path_pub = rospy.Publisher(
            'planned_path',
            Path,
            queue_size=10
        )
        
        self.current_target_pub = rospy.Publisher(
            'current_target',
            PoseStamped,
            queue_size=10
        )
        
        self.status_pub = rospy.Publisher(
            'planner_status',
            String,
            queue_size=10
        )
        
        # 订阅器
        self.pose_sub = rospy.Subscriber(
            'localization/current_pose',  # 来自定位模块
            PoseStamped,
            self.pose_callback
        )
        
        self.mode_sub = rospy.Subscriber(
            'planning_mode',
            String,
            self.mode_callback
        )
        
        self.target_sub = rospy.Subscriber(
            'target_pose',
            PoseStamped,
            self.target_callback
        )
        
        # 定时器
        self.plan_timer = rospy.Timer(
            rospy.Duration(1.0/self.plan_rate),
            self.planning_loop
        )
        
        rospy.loginfo("🗺️  轨迹规划器已启动")
        rospy.loginfo(f"⚙️  飞行高度: {self.altitude}m")
        rospy.loginfo(f"🎯 探索半径: {self.exploration_radius}m")
        rospy.loginfo(f"📊 规划频率: {self.plan_rate} Hz")
    
    def pose_callback(self, msg):
        """接收当前位置"""
        self.current_pose = msg
    
    def mode_callback(self, msg):
        """切换规划模式"""
        mode_str = msg.data.upper()
        
        if mode_str == "EXPLORATION":
            self.current_mode = PlanningMode.EXPLORATION
            self.current_waypoint_index = 0
            rospy.loginfo("🔍 切换到探索模式")
        elif mode_str == "WAYPOINT":
            self.current_mode = PlanningMode.WAYPOINT
            self.current_waypoint_index = 0
            rospy.loginfo("📍 切换到航点模式")
        elif mode_str == "TARGET":
            self.current_mode = PlanningMode.TARGET
            rospy.loginfo("🎯 切换到目标模式")
        elif mode_str == "IDLE":
            self.current_mode = PlanningMode.IDLE
            rospy.loginfo("⏸️  切换到空闲模式")
        else:
            rospy.logwarn(f"❌ 未知模式: {mode_str}")
    
    def target_callback(self, msg):
        """接收目标位置"""
        self.target_pose = msg
        rospy.loginfo(f"🎯 接收到目标: [{msg.pose.position.x:.1f}, {msg.pose.position.y:.1f}, {msg.pose.position.z:.1f}]")
    
    def generate_exploration_waypoints(self):
        """生成探索航点"""
        waypoints = []
        
        # 方形螺旋探索模式
        radius = self.exploration_radius
        
        # 中心起点
        waypoints.append([0, 0, self.altitude])
        
        # 螺旋扩展
        for r in np.arange(1, radius + 1, 2):
            # 右
            waypoints.append([r, 0, self.altitude])
            # 上
            waypoints.append([r, r, self.altitude])
            # 左
            waypoints.append([-r, r, self.altitude])
            # 下
            waypoints.append([-r, -r, self.altitude])
            # 右下
            waypoints.append([r, -r, self.altitude])
        
        rospy.loginfo(f"📍 生成 {len(waypoints)} 个探索航点")
        return waypoints
    
    def generate_path_to_target(self, target):
        """生成到目标的路径"""
        if self.current_pose is None:
            return None
        
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        
        # 简单直线路径 (可以改为A*或RRT等复杂算法)
        start_pos = np.array([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ])
        
        target_pos = np.array([
            target[0], target[1], target[2]
        ])
        
        # 分段路径
        distance = np.linalg.norm(target_pos - start_pos)
        num_points = max(int(distance / self.waypoint_spacing), 2)
        
        for i in range(num_points + 1):
            alpha = i / num_points
            pos = start_pos + alpha * (target_pos - start_pos)
            
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            pose.pose.position.z = pos[2]
            pose.pose.orientation.w = 1.0
            
            path.poses.append(pose)
        
        return path
    
    def get_current_target_waypoint(self):
        """获取当前目标航点"""
        if self.current_mode == PlanningMode.EXPLORATION:
            if self.current_waypoint_index < len(self.exploration_waypoints):
                return self.exploration_waypoints[self.current_waypoint_index]
        elif self.current_mode == PlanningMode.TARGET and self.target_pose:
            return [
                self.target_pose.pose.position.x,
                self.target_pose.pose.position.y,
                self.target_pose.pose.position.z
            ]
        
        return None
    
    def check_waypoint_reached(self, target_waypoint, threshold=0.5):
        """检查是否到达航点"""
        if self.current_pose is None or target_waypoint is None:
            return False
        
        current_pos = np.array([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ])
        
        target_pos = np.array(target_waypoint)
        distance = np.linalg.norm(current_pos - target_pos)
        
        return distance < threshold
    
    def planning_loop(self, event):
        """主规划循环"""
        if self.current_pose is None:
            return
        
        # 发布状态
        status_msg = String()
        status_msg.data = f"Mode: {self.current_mode.name}, Waypoint: {self.current_waypoint_index}"
        self.status_pub.publish(status_msg)
        
        if self.current_mode == PlanningMode.IDLE:
            return
        
        # 获取当前目标
        target_waypoint = self.get_current_target_waypoint()
        if target_waypoint is None:
            return
        
        # 生成路径
        path = self.generate_path_to_target(target_waypoint)
        if path:
            self.current_path = path
            self.path_pub.publish(path)
        
        # 发布当前目标点
        target_msg = PoseStamped()
        target_msg.header.stamp = rospy.Time.now()
        target_msg.header.frame_id = "map"
        target_msg.pose.position.x = target_waypoint[0]
        target_msg.pose.position.y = target_waypoint[1]
        target_msg.pose.position.z = target_waypoint[2]
        target_msg.pose.orientation.w = 1.0
        self.current_target_pub.publish(target_msg)
        
        # 检查航点完成
        if self.check_waypoint_reached(target_waypoint):
            if self.current_mode == PlanningMode.EXPLORATION:
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.exploration_waypoints):
                    rospy.loginfo("🎉 探索任务完成!")
                    self.current_mode = PlanningMode.IDLE
                else:
                    wp = target_waypoint
                    rospy.loginfo(f"✅ 到达航点 {self.current_waypoint_index-1}: [{wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f}]")
            elif self.current_mode == PlanningMode.TARGET:
                rospy.loginfo("🎯 到达目标位置!")
                self.current_mode = PlanningMode.IDLE
        
        # 状态日志
        if rospy.get_time() % 5 < 0.1:
            pos = self.current_pose.pose.position
            rospy.loginfo(f"📍 当前位置: [{pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}]")
            if target_waypoint:
                rospy.loginfo(f"🎯 目标航点: [{target_waypoint[0]:.1f}, {target_waypoint[1]:.1f}, {target_waypoint[2]:.1f}]")
    
    def run(self):
        """运行规划器"""
        rospy.loginfo("🗺️  等待定位数据...")
        
        # 等待定位数据
        while not rospy.is_shutdown() and self.current_pose is None:
            rospy.sleep(0.1)
        
        rospy.loginfo("✅ 定位数据已接收，轨迹规划器就绪")
        rospy.loginfo("💡 发送规划模式到 /planning_mode 话题:")
        rospy.loginfo("   - EXPLORATION: 自主探索")
        rospy.loginfo("   - TARGET: 目标导航")
        rospy.loginfo("   - IDLE: 停止规划")
        
        rospy.spin()

if __name__ == '__main__':
    try:
        planner = TrajectoryPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 轨迹规划器关闭")
    except Exception as e:
        rospy.logerr(f"❌ 轨迹规划器异常: {e}") 