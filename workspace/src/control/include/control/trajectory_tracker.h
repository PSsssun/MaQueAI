#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>
#include <atomic>

#include "control/pid_controller.h"
#include "control/safety_monitor.h"

namespace control {

/**
 * @brief 轨迹跟踪控制器
 * 
 * 高性能实时控制器，实现基于PID的轨迹跟踪
 */
class TrajectoryTracker {
public:
    explicit TrajectoryTracker(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~TrajectoryTracker() = default;

    /**
     * @brief 运行控制器主循环
     */
    void run();

    /**
     * @brief 启动轨迹跟踪
     */
    void startTracking();

    /**
     * @brief 停止轨迹跟踪
     */
    void stopTracking();

    /**
     * @brief 紧急停止
     */
    void emergencyStop();

    /**
     * @brief 检查控制器状态
     */
    bool isHealthy() const;

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // 订阅器和发布器
    ros::Subscriber pose_sub_;          // 当前位置
    ros::Subscriber path_sub_;          // 规划路径
    ros::Subscriber target_sub_;        // 当前目标点
    ros::Subscriber cmd_sub_;           // 控制命令
    
    ros::Publisher control_pub_;        // 控制输出
    ros::Publisher status_pub_;         // 状态发布
    ros::Publisher debug_pub_;          // 调试信息
    
    // 定时器
    ros::Timer control_timer_;
    
    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 控制器组件
    std::unique_ptr<PIDController> position_controller_;
    std::unique_ptr<PIDController> velocity_controller_;
    std::unique_ptr<SafetyMonitor> safety_monitor_;
    
    // 参数
    double control_frequency_;          // 控制频率
    double max_velocity_;              // 最大速度
    double max_acceleration_;          // 最大加速度
    double position_tolerance_;        // 位置容差
    double velocity_tolerance_;        // 速度容差
    std::string map_frame_;            // 地图坐标系
    std::string base_frame_;           // 机体坐标系
    
    // 状态变量
    std::atomic<bool> tracking_enabled_;
    std::atomic<bool> emergency_stop_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::TwistStamped current_velocity_;
    nav_msgs::Path current_path_;
    geometry_msgs::PoseStamped current_target_;
    size_t current_waypoint_index_;
    
    // 控制状态
    Eigen::Vector3d position_error_;
    Eigen::Vector3d velocity_error_;
    Eigen::Vector3d control_output_;
    ros::Time last_control_time_;
    
    // 轨迹信息
    std::vector<Eigen::Vector3d> waypoints_;
    std::vector<double> waypoint_times_;
    double trajectory_start_time_;
    
    // 回调函数
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void commandCallback(const std_msgs::String::ConstPtr& msg);
    void controlTimerCallback(const ros::TimerEvent& event);
    
    // 控制算法
    void updateControl();
    void computePositionError();
    void computeVelocityError();
    void computeControlOutput();
    void publishControlOutput();
    void publishStatus();
    
    // 轨迹处理
    void processNewPath(const nav_msgs::Path& path);
    bool findNearestWaypoint(size_t& waypoint_index) const;
    geometry_msgs::PoseStamped interpolateTarget() const;
    bool isWaypointReached(const Eigen::Vector3d& waypoint) const;
    
    // 工具函数
    static Eigen::Vector3d poseToVector3d(const geometry_msgs::Point& point);
    static geometry_msgs::Point vector3dToPoint(const Eigen::Vector3d& vec);
    static double calculateDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
    
    // 安全检查
    bool performSafetyChecks() const;
    void limitControlOutput(Eigen::Vector3d& output) const;
};

} // namespace control 