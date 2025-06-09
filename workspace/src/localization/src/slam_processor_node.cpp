#include "localization/slam_processor.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "slam_processor");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    try {
        localization::SLAMProcessor processor(nh, pnh);
        processor.run();
    } catch (const std::exception& e) {
        ROS_FATAL("❌ SLAM处理器异常: %s", e.what());
        return -1;
    }
    
    return 0;
} 