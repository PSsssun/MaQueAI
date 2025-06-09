#pragma once

#include <eigen3/Eigen/Dense>
#include <chrono>
#include <memory>

namespace control {

/**
 * @brief 高性能3D PID控制器
 * 
 * 支持位置、速度和加速度控制的多维PID控制器
 */
class PIDController {
public:
    /**
     * @brief PID参数结构
     */
    struct PIDParams {
        Eigen::Vector3d kp{1.0, 1.0, 1.0};      // 比例增益
        Eigen::Vector3d ki{0.1, 0.1, 0.1};      // 积分增益  
        Eigen::Vector3d kd{0.05, 0.05, 0.05};   // 微分增益
        
        Eigen::Vector3d max_output{10.0, 10.0, 10.0};     // 输出限制
        Eigen::Vector3d max_integral{5.0, 5.0, 5.0};      // 积分限制
        
        double deadband{0.01};                   // 死区
        bool enable_derivative_filter{true};     // 微分滤波
        double derivative_filter_coeff{0.8};     // 微分滤波系数
    };

    explicit PIDController(const PIDParams& params = PIDParams{});
    ~PIDController() = default;

    /**
     * @brief 计算PID控制输出
     * @param error 当前误差
     * @param dt 时间步长
     * @return 控制输出
     */
    Eigen::Vector3d compute(const Eigen::Vector3d& error, double dt);

    /**
     * @brief 重置控制器状态
     */
    void reset();

    /**
     * @brief 更新PID参数
     */
    void updateParams(const PIDParams& params);

    /**
     * @brief 获取当前参数
     */
    const PIDParams& getParams() const { return params_; }

    /**
     * @brief 获取积分项
     */
    const Eigen::Vector3d& getIntegral() const { return integral_; }

    /**
     * @brief 获取微分项
     */
    const Eigen::Vector3d& getDerivative() const { return derivative_; }

    /**
     * @brief 设置积分限制
     */
    void setIntegralLimits(const Eigen::Vector3d& limits);

    /**
     * @brief 设置输出限制
     */
    void setOutputLimits(const Eigen::Vector3d& limits);

private:
    PIDParams params_;
    
    // 控制器状态
    Eigen::Vector3d integral_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d previous_error_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d derivative_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d filtered_derivative_{Eigen::Vector3d::Zero()};
    
    bool first_run_{true};
    
    // 工具函数
    Eigen::Vector3d saturate(const Eigen::Vector3d& input, const Eigen::Vector3d& limits) const;
    Eigen::Vector3d applyDeadband(const Eigen::Vector3d& input, double deadband) const;
};

/**
 * @brief 级联PID控制器
 * 
 * 实现位置-速度级联控制
 */
class CascadePIDController {
public:
    struct CascadeParams {
        PIDController::PIDParams position_params;
        PIDController::PIDParams velocity_params;
        
        Eigen::Vector3d max_velocity{5.0, 5.0, 2.0};      // 最大速度限制
        Eigen::Vector3d max_acceleration{10.0, 10.0, 5.0}; // 最大加速度限制
    };

    explicit CascadePIDController(const CascadeParams& params = CascadeParams{});
    ~CascadePIDController() = default;

    /**
     * @brief 计算级联控制输出
     * @param position_error 位置误差
     * @param velocity_error 速度误差  
     * @param dt 时间步长
     * @return 加速度控制输出
     */
    Eigen::Vector3d compute(const Eigen::Vector3d& position_error,
                           const Eigen::Vector3d& velocity_error,
                           double dt);

    /**
     * @brief 重置控制器
     */
    void reset();

    /**
     * @brief 更新参数
     */
    void updateParams(const CascadeParams& params);

    /**
     * @brief 获取位置控制器
     */
    PIDController& getPositionController() { return position_controller_; }

    /**
     * @brief 获取速度控制器
     */
    PIDController& getVelocityController() { return velocity_controller_; }

private:
    CascadeParams params_;
    PIDController position_controller_;
    PIDController velocity_controller_;
    
    Eigen::Vector3d saturate(const Eigen::Vector3d& input, const Eigen::Vector3d& limits) const;
};

} // namespace control 