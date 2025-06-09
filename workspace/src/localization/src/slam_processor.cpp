#include "localization/slam_processor.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>

namespace localization {

SLAMProcessor::SLAMProcessor(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), pose_initialized_(false) {
    
    // 加载参数
    pnh_.param("filter_alpha", filter_alpha_, 0.8);
    pnh_.param("publish_rate", publish_rate_, 50.0);
    pnh_.param("timeout_threshold", timeout_threshold_, 0.5);
    pnh_.param("map_frame", map_frame_, std::string("map"));
    pnh_.param("base_frame", base_frame_, std::string("base_link"));
    
    // 初始化TF2
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 初始化发布器
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("localization/current_pose", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("localization/filtered_odom", 10);
    status_pub_ = nh_.advertise<std_msgs::String>("localization/status", 10);
    
    // 初始化订阅器
    slam_sub_ = nh_.subscribe("/Odometry", 10, &SLAMProcessor::slamCallback, this);
    
    // 初始化定时器
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), 
                                   &SLAMProcessor::publishTimerCallback, this);
    
    // 初始化滤波位置
    filtered_pose_.position.x = 0.0;
    filtered_pose_.position.y = 0.0;
    filtered_pose_.position.z = 0.0;
    filtered_pose_.orientation.w = 1.0;
    filtered_pose_.orientation.x = 0.0;
    filtered_pose_.orientation.y = 0.0;
    filtered_pose_.orientation.z = 0.0;
    
    ROS_INFO("📍 SLAM位置处理器已启动");
    ROS_INFO("⚙️  滤波系数: %.2f", filter_alpha_);
    ROS_INFO("📊 发布频率: %.1f Hz", publish_rate_);
    ROS_INFO("🗺️  坐标系: %s -> %s", map_frame_.c_str(), base_frame_.c_str());
}

void SLAMProcessor::run() {
    ROS_INFO("📍 等待SLAM数据...");
    
    // 等待SLAM数据
    while (ros::ok() && !current_odom_) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    
    ROS_INFO("✅ SLAM数据已接收，开始位置处理");
    ros::spin();
}

void SLAMProcessor::slamCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom_ = msg;
    last_update_time_ = msg->header.stamp;
    
    // 滤波处理位置
    filterPose(msg->pose.pose);
    
    // 更新位置历史
    Eigen::Vector3d current_pos = poseToVector3d(filtered_pose_.position);
    updatePositionHistory(msg->header.stamp, current_pos);
}

void SLAMProcessor::filterPose(const geometry_msgs::Pose& new_pose) {
    if (!pose_initialized_) {
        // 初始化
        filtered_pose_ = new_pose;
        pose_initialized_ = true;
        ROS_INFO("📍 位置滤波器已初始化");
    } else {
        // 低通滤波 - 位置
        filtered_pose_.position.x = filter_alpha_ * filtered_pose_.position.x + 
                                   (1.0 - filter_alpha_) * new_pose.position.x;
        filtered_pose_.position.y = filter_alpha_ * filtered_pose_.position.y + 
                                   (1.0 - filter_alpha_) * new_pose.position.y;
        filtered_pose_.position.z = filter_alpha_ * filtered_pose_.position.z + 
                                   (1.0 - filter_alpha_) * new_pose.position.z;
        
        // 姿态直接更新 (可以改为四元数插值)
        filtered_pose_.orientation = new_pose.orientation;
    }
}

void SLAMProcessor::publishTimerCallback(const ros::TimerEvent& event) {
    if (!current_odom_ || !pose_initialized_) {
        return;
    }
    
    ros::Time current_time = ros::Time::now();
    
    // 检查数据时效性
    if ((current_time - last_update_time_).toSec() > timeout_threshold_) {
        ROS_WARN_THROTTLE(5, "⚠️  SLAM数据过期 (%.2f秒)", 
                         (current_time - last_update_time_).toSec());
        return;
    }
    
    // 发布位置和TF
    publishPose(current_time);
    publishTransform(current_time);
    
    // 发布状态
    std_msgs::String status_msg;
    status_msg.data = "HEALTHY";
    status_pub_.publish(status_msg);
    
    // 定期日志
    static int log_counter = 0;
    if (++log_counter % static_cast<int>(publish_rate_ * 5) == 0) {
        ROS_INFO("📍 当前位置: [%.2f, %.2f, %.2f]", 
                filtered_pose_.position.x, 
                filtered_pose_.position.y, 
                filtered_pose_.position.z);
    }
}

void SLAMProcessor::publishPose(const ros::Time& stamp) {
    // 发布当前位置 (给规划模块)
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = filtered_pose_;
    pose_pub_.publish(pose_msg);
    
    // 发布滤波后的里程计 (给可视化和控制)
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose = filtered_pose_;
    
    // 估计速度
    Eigen::Vector3d velocity = estimateVelocity();
    odom_msg.twist.twist.linear.x = velocity.x();
    odom_msg.twist.twist.linear.y = velocity.y();
    odom_msg.twist.twist.linear.z = velocity.z();
    
    // 复制原始协方差 (如果有)
    if (current_odom_->pose.covariance[0] > 0) {
        odom_msg.pose.covariance = current_odom_->pose.covariance;
    }
    
    odom_pub_.publish(odom_msg);
}

void SLAMProcessor::publishTransform(const ros::Time& stamp) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = base_frame_;
    
    transform.transform.translation.x = filtered_pose_.position.x;
    transform.transform.translation.y = filtered_pose_.position.y;
    transform.transform.translation.z = filtered_pose_.position.z;
    
    transform.transform.rotation = filtered_pose_.orientation;
    
    tf_broadcaster_->sendTransform(transform);
}

void SLAMProcessor::updatePositionHistory(const ros::Time& stamp, const Eigen::Vector3d& position) {
    position_history_.emplace_back(stamp, position);
    
    // 限制历史大小
    while (position_history_.size() > MAX_HISTORY_SIZE) {
        position_history_.pop_front();
    }
}

Eigen::Vector3d SLAMProcessor::estimateVelocity() const {
    if (position_history_.size() < 2) {
        return Eigen::Vector3d::Zero();
    }
    
    // 使用最近两个位置估计速度
    const auto& latest = position_history_.back();
    const auto& previous = position_history_[position_history_.size() - 2];
    
    double dt = (latest.first - previous.first).toSec();
    if (dt <= 0) {
        return Eigen::Vector3d::Zero();
    }
    
    return (latest.second - previous.second) / dt;
}

geometry_msgs::PoseStamped SLAMProcessor::getCurrentPose() const {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = map_frame_;
    pose.pose = filtered_pose_;
    return pose;
}

bool SLAMProcessor::isLocalizationHealthy() const {
    if (!current_odom_ || !pose_initialized_) {
        return false;
    }
    
    double time_since_update = (ros::Time::now() - last_update_time_).toSec();
    return time_since_update < timeout_threshold_;
}

// 工具函数实现
Eigen::Vector3d SLAMProcessor::poseToVector3d(const geometry_msgs::Point& point) {
    return Eigen::Vector3d(point.x, point.y, point.z);
}

geometry_msgs::Point SLAMProcessor::vector3dToPoint(const Eigen::Vector3d& vec) {
    geometry_msgs::Point point;
    point.x = vec.x();
    point.y = vec.y();
    point.z = vec.z();
    return point;
}

} // namespace localization 