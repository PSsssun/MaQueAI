#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

// MAVROS消息
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <atomic>
#include <mutex>

namespace interface {

/**
 * @brief MAVROS通信桥接器
 * 
 * 高性能实时桥接器，连接内部控制系统与PX4飞控
 */
class MAVROSBridge {
public:
    /**
     * @brief 飞行模式枚举
     */
    enum class FlightMode {
        MANUAL,
        STABILIZED,
        ACRO,
        RATTITUDE,
        ALTCTL,
        POSCTL,
        AUTO_LOITER,
        AUTO_RTL,
        AUTO_MISSION,
        OFFBOARD
    };

    /**
     * @brief 飞行状态枚举
     */
    enum class FlightState {
        DISARMED,
        ARMED,
        TAKEOFF,
        HOVER,
        MISSION,
        LANDING,
        LANDED,
        ERROR
    };

    explicit MAVROSBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~MAVROSBridge() = default;

    /**
     * @brief 运行桥接器主循环
     */
    void run();

    /**
     * @brief 解锁飞行器
     */
    bool arm();

    /**
     * @brief 锁定飞行器  
     */
    bool disarm();

    /**
     * @brief 起飞到指定高度
     */
    bool takeoff(double altitude);

    /**
     * @brief 降落
     */
    bool land();

    /**
     * @brief 切换到OFFBOARD模式
     */
    bool setOffboardMode();

    /**
     * @brief 紧急返航
     */
    bool returnToLaunch();

    /**
     * @brief 检查连接状态
     */
    bool isConnected() const;

    /**
     * @brief 检查解锁状态
     */
    bool isArmed() const;

    /**
     * @brief 获取当前模式
     */
    FlightMode getCurrentMode() const;

    /**
     * @brief 获取当前状态
     */
    FlightState getCurrentState() const;

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // 订阅器
    ros::Subscriber control_cmd_sub_;       // 控制指令
    ros::Subscriber slam_odom_sub_;        // SLAM里程计
    ros::Subscriber mavros_state_sub_;     // MAVROS状态
    ros::Subscriber mavros_extended_state_sub_; // 扩展状态
    ros::Subscriber mavros_pose_sub_;      // 当前位置
    
    // 发布器
    ros::Publisher mavros_setpoint_pub_;   // 位置设定点
    ros::Publisher mavros_velocity_pub_;   // 速度设定点
    ros::Publisher mavros_attitude_pub_;   // 姿态设定点
    ros::Publisher vision_pose_pub_;       // 视觉位置
    ros::Publisher status_pub_;            // 状态发布
    
    // 服务客户端
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient takeoff_client_;
    ros::ServiceClient land_client_;
    
    // 定时器
    ros::Timer heartbeat_timer_;
    ros::Timer status_timer_;
    
    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 参数
    double heartbeat_rate_;                // 心跳频率
    double status_rate_;                   // 状态发布频率
    double connection_timeout_;            // 连接超时
    std::string map_frame_;               // 地图坐标系
    std::string base_frame_;              // 机体坐标系
    
    // 状态变量
    std::atomic<bool> connected_;
    std::atomic<bool> armed_;
    mavros_msgs::State current_mavros_state_;
    mavros_msgs::ExtendedState current_extended_state_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::TwistStamped current_velocity_;
    nav_msgs::Odometry current_slam_odom_;
    
    FlightMode current_mode_;
    FlightState current_state_;
    
    // 控制指令
    mavros_msgs::PositionTarget position_target_;
    mavros_msgs::AttitudeTarget attitude_target_;
    
    // 线程安全
    mutable std::mutex state_mutex_;
    mutable std::mutex command_mutex_;
    
    // 统计信息
    ros::Time last_heartbeat_time_;
    int command_sequence_;
    
    // 回调函数
    void controlCommandCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void slamOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
    void mavrosExtendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& msg);
    void mavrosPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    void heartbeatTimerCallback(const ros::TimerEvent& event);
    void statusTimerCallback(const ros::TimerEvent& event);
    
    // 控制处理
    void processPositionCommand(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
    void processVelocityCommand(const Eigen::Vector3d& velocity);
    void processAttitudeCommand(const Eigen::Vector4d& attitude, double thrust);
    
    // 状态管理
    void updateFlightState();
    void publishStatus();
    void publishVisionPose();
    
    // 安全检查
    bool performSafetyChecks() const;
    bool isReadyForOffboard() const;
    
    // 坐标转换
    geometry_msgs::PoseStamped transformPoseENU2NED(const geometry_msgs::PoseStamped& pose_enu) const;
    geometry_msgs::PoseStamped transformPoseNED2ENU(const geometry_msgs::PoseStamped& pose_ned) const;
    geometry_msgs::TwistStamped transformTwistENU2NED(const geometry_msgs::TwistStamped& twist_enu) const;
    
    // 工具函数
    static FlightMode mavrosModeToFlightMode(const std::string& mode_str);
    static std::string flightModeToString(FlightMode mode);
    static std::string flightStateToString(FlightState state);
    
    // MAVROS服务调用
    bool callArmingService(bool arm);
    bool callSetModeService(const std::string& mode);
    bool callTakeoffService(double altitude);
    bool callLandService();
};

} // namespace interface 