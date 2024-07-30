#pragma once
#include <glog/logging.h>
#include <livox_ros_driver/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include "sensors/point_types.hh"
#include "tools/lidar_utils.hh"

namespace ctlio {
/**
 * @brief 处理多种不同的雷达点云
 */
enum class LidarType {
    AVIA = 1,
    VELO32,
    OUST64,
    ROBOSENS16,
    PANDAR,
    LS,  // 镭神
};

class LidarProcess {
   public:
    LidarProcess() = default;
    ~LidarProcess() = default;

    void LoadYaml(const std::string& yaml_file);

    // 处理livox雷达
    // void process(const livox_ros_driver::CustomMsg::Ptr& msg, FullPointCloud& pcl_out);
    void process(const livox_ros_driver::CustomMsg::Ptr& msg, std::vector<point3D>& pcl_out);
    // 处理标准的点云雷达
    void process(const sensor_msgs::PointCloud2::Ptr& msg, std::vector<point3D>& pcl_out);
    double getTimeSpan() const {
        return timespan_;
    }

   private:
    void AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr& msg, std::vector<point3D>& pcl_out);
    void VelodyneHandler(const sensor_msgs::PointCloud2::Ptr& msg, std::vector<point3D>& pcl_out);
    void OusterHandler(const sensor_msgs::PointCloud2::Ptr& msg, std::vector<point3D>& pcl_out);
    void PandarHandler(const sensor_msgs::PointCloud2::Ptr& msg, std::vector<point3D>& pcl_out);
    void RobosenseHandler(const sensor_msgs::PointCloud2::Ptr& msg, std::vector<point3D>& pcl_out);
    void LslidarHandler(const sensor_msgs::PointCloud2::Ptr& msg, std::vector<point3D>& pcl_out);

   private:
    LidarType lidar_type = LidarType::AVIA;
    int point_filter_num = 1;
    float blind = 0.1;
    double timespan_ = 0.0;
};
}  // namespace ctlio