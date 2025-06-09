#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SLAM位置处理器

功能: 处理LIVO SLAM输出，提供标准化的位置信息给规划和控制模块
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class SLAMProcessor:
    def __init__(self):
        rospy.init_node('slam_processor', anonymous=True)
        
        # 参数配置
        self.filter_alpha = rospy.get_param('~filter_alpha', 0.8)  # 滤波系数
        self.publish_rate = rospy.get_param('~publish_rate', 50.0)  # 发布频率
        
        # 状态变量
        self.current_odom = None
        self.filtered_pose = None
        self.last_update_time = rospy.Time(0)
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 发布器
        self.pose_pub = rospy.Publisher(
            'current_pose', 
            PoseStamped, 
            queue_size=10
        )
        
        self.odom_pub = rospy.Publisher(
            'filtered_odom',
            Odometry,
            queue_size=10
        )
        
        # 订阅器 - LIVO SLAM输出
        self.slam_sub = rospy.Subscriber(
            '/Odometry',  # LIVO输出
            Odometry,
            self.slam_callback
        )
        
        # 定时发布
        self.timer = rospy.Timer(
            rospy.Duration(1.0/self.publish_rate),
            self.publish_pose
        )
        
        rospy.loginfo("📍 SLAM位置处理器已启动")
        rospy.loginfo(f"⚙️  滤波系数: {self.filter_alpha}")
        rospy.loginfo(f"📊 发布频率: {self.publish_rate} Hz")
    
    def slam_callback(self, msg):
        """接收LIVO SLAM位置数据"""
        self.current_odom = msg
        self.last_update_time = msg.header.stamp
        
        # 简单低通滤波
        if self.filtered_pose is None:
            self.filtered_pose = msg.pose.pose
        else:
            # 位置滤波
            self.filtered_pose.position.x = (
                self.filter_alpha * self.filtered_pose.position.x +
                (1 - self.filter_alpha) * msg.pose.pose.position.x
            )
            self.filtered_pose.position.y = (
                self.filter_alpha * self.filtered_pose.position.y +
                (1 - self.filter_alpha) * msg.pose.pose.position.y
            )
            self.filtered_pose.position.z = (
                self.filter_alpha * self.filtered_pose.position.z +
                (1 - self.filter_alpha) * msg.pose.pose.position.z
            )
            
            # 姿态更新 (可以更复杂的四元数滤波)
            self.filtered_pose.orientation = msg.pose.pose.orientation
    
    def publish_pose(self, event):
        """发布标准化位置信息"""
        if self.current_odom is None or self.filtered_pose is None:
            return
        
        current_time = rospy.Time.now()
        
        # 检查数据时效性
        if (current_time - self.last_update_time).to_sec() > 0.5:
            rospy.logwarn_throttle(5, "⚠️  SLAM数据过期")
            return
        
        # 发布当前位置 (给规划模块)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = "map"
        pose_msg.pose = self.filtered_pose
        self.pose_pub.publish(pose_msg)
        
        # 发布滤波后的里程计 (给可视化)
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose = self.filtered_pose
        if self.current_odom.twist:
            odom_msg.twist = self.current_odom.twist
        self.odom_pub.publish(odom_msg)
        
        # 发布TF变换
        self.publish_transform(current_time)
        
        # 状态日志
        if rospy.get_time() % 5 < 0.1:
            pos = self.filtered_pose.position
            rospy.loginfo(f"📍 当前位置: [{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}]")
    
    def publish_transform(self, stamp):
        """发布坐标变换"""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        
        t.transform.translation.x = self.filtered_pose.position.x
        t.transform.translation.y = self.filtered_pose.position.y
        t.transform.translation.z = self.filtered_pose.position.z
        
        t.transform.rotation = self.filtered_pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    def get_current_pose(self):
        """获取当前位置 (供其他模块调用)"""
        return self.filtered_pose if self.filtered_pose else None
    
    def run(self):
        """运行处理器"""
        rospy.loginfo("📍 等待SLAM数据...")
        
        # 等待SLAM数据
        while not rospy.is_shutdown() and self.current_odom is None:
            rospy.sleep(0.1)
        
        rospy.loginfo("✅ SLAM数据已接收，开始位置处理")
        rospy.spin()

if __name__ == '__main__':
    try:
        processor = SLAMProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 SLAM处理器关闭")
    except Exception as e:
        rospy.logerr(f"❌ SLAM处理器异常: {e}") 