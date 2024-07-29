#include <livox_ros_driver/CustomMsg.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <future>
#include <opencv2/core.hpp>
// #include "segment/segment.hpp"
/**
 * @brief 特征提取模块
 */
namespace ctlio {
class LidarFeatureExtractor {
   public:
    LidarFeatureExtractor() = default;

   private:
};
}  // namespace ctlio