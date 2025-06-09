#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <deque>

namespace localization {

/**
 * @brief SLAM位置处理器
 * 
 * 处理LIVO SLAM输出，提供标准化的高频位置信息
 */
class SLAMProcessor {
public:
    explicit SLAMProcessor(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~SLAMProcessor() = default;

    /**
     * @brief 运行处理器主循环
     */
    void run();

    /**
     * @brief 获取当前滤波后的位置
     */
    geometry_msgs::PoseStamped getCurrentPose() const;

    /**
     * @brief 检查定位系统状态
     */
    bool isLocalizationHealthy() const;

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // 订阅器和发布器
    ros::Subscriber slam_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher status_pub_;
    
    // TF2
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 定时器
    ros::Timer publish_timer_;
    
    // 参数
    double filter_alpha_;       // 滤波系数
    double publish_rate_;       // 发布频率
    double timeout_threshold_;  // 数据超时阈值
    std::string map_frame_;     // 地图坐标系
    std::string base_frame_;    // 机体坐标系
    
    // 状态变量
    nav_msgs::Odometry::ConstPtr current_odom_;
    geometry_msgs::Pose filtered_pose_;
    ros::Time last_update_time_;
    bool pose_initialized_;
    
    // 位置历史 (用于速度估计)
    std::deque<std::pair<ros::Time, Eigen::Vector3d>> position_history_;
    static constexpr size_t MAX_HISTORY_SIZE = 10;
    
    // 回调函数
    void slamCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void publishTimerCallback(const ros::TimerEvent& event);
    
    // 处理函数
    void filterPose(const geometry_msgs::Pose& new_pose);
    void publishPose(const ros::Time& stamp);
    void publishTransform(const ros::Time& stamp);
    void updatePositionHistory(const ros::Time& stamp, const Eigen::Vector3d& position);
    Eigen::Vector3d estimateVelocity() const;
    
    // 工具函数
    static Eigen::Vector3d poseToVector3d(const geometry_msgs::Point& point);
    static geometry_msgs::Point vector3dToPoint(const Eigen::Vector3d& vec);
};

} // namespace localization 