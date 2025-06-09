# 定位系统状态消息

# 消息头
Header header

# 定位状态
bool is_healthy            # 系统是否健康
bool pose_initialized      # 位置是否已初始化
float64 last_update_age    # 上次更新距今时间(秒)

# 精度信息
float64 position_accuracy  # 位置精度(米)
float64 orientation_accuracy # 姿态精度(弧度)

# 统计信息
int32 total_measurements   # 总测量次数
float64 average_frequency  # 平均频率(Hz)

# 错误信息
string error_message       # 错误描述 