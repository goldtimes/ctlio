#pragma once
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <sophus/se3.hpp>
#include <string>
#include <vector>
#include "ekf_utils/static_imu_init.hh"
#include "eskf/eskf.hpp"
#include "factor/factors.hh"
#include "lio/lio_utils.hh"
#include "sensors/imu.hh"
#include "sensors/point_types.hh"
#include "tools/lidar_utils.hh"
#include "tools/timer/timer.hh"
#include "voxel/voxel_map.hpp"
namespace ctlio {
// lio的配置
struct LioOptions {
    // 体素大小
    double size_voxel_map;
    double min_distance_points;
    // 体素中的最大点数
    double max_num_points_in_voxel;
    // 体素中的点到当前雷达位姿的距离，大于这个距离的点的voxel被删除
    double max_distance;

    double sampling_rate;
    double surf_res;

    bool point_to_plane_with_distortion;
    ICPMODEL icp_model;
    MotionCompensation motion_compensation;
    int max_num_iteration;

    double weight_alpha;
    double weight_neighborhood;
    double max_dist_to_plane_icp;
    int init_num_frames;
    int voxel_neighborhood;
    int threshold_voxel_occupancy;

    // neighbors的阈值
    int max_number_neighbors;
    int min_number_neighbors;

    bool estimate_normal_from_neighborhood;
    int power_planarity;
    int num_closest_neighbors;
    // 构造的残差数
    int max_num_residuals;
    int min_num_resdiuals;

    double thres_rotation_norm;
    double thres_trans_norm;
};

class LidarOdom {
   public:
    using pair_distance_t = std::tuple<double, Eigen::Vector3d, Voxel>;
    struct comparator {
        bool operator()(const pair_distance_t& left, const pair_distance_t& right) const {
            return std::get<0>(left) < std::get<0>(right);
        }
    };
    using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, comparator>;
    LidarOdom();
    ~LidarOdom() = default;
    bool init(const std::string& config_file);
    void pushImu(IMUPtr& imu_msg);
    void pushLidar(const std::vector<point3D>&, std::pair<double, double> data);
    // 死循环函数，一直处理传感器数据
    void run();

    void setFunc(std::function<bool(std::string& topic_name, CloudPtr& cloud, double time)>& func) {
        pub_cloud_to_ros = func;
    }
    void setFunc(std::function<bool(std::string& topic_name, Sophus::SE3d& pose, double time)>& func) {
        pub_pose_to_ros = func;
    }
    void setFunc(std::function<bool(std::string& topic_name, double time1, double time2)>& func) {
        pub_data_to_ros = func;
    }

   private:
    void loadOptions(const std::string& config_file);
    std::vector<MeasureGroup> getMeasurments();
    void processMeasurements(MeasureGroup& meas);
    void TryInitIMU();
    void Predict();

    void stateInitialization();
    std::shared_ptr<CloudFrame> BuildFrame(const std::vector<point3D>& points_lidar,
                                           std::shared_ptr<State> current_state, double frame_begin_time,
                                           double frame_end_time);
    void Undistort(std::vector<point3D>& points);

    void PoseEstimation(std::shared_ptr<CloudFrame> frame);

    void map_incremental(std::shared_ptr<CloudFrame> frame);
    void lasermap_fov_segment();

    void AddPointToMap(VoxelHashMap& map, const Eigen::Vector3d& point, const double intensity, double voxel_size,
                       int max_num_in_voxel, double min_distance_points, int min_num_points);
    void AddPointToCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const Eigen::Vector3d& point,
                         const double intensity);
    void Optimize(std::shared_ptr<CloudFrame> frame);

    void AddSurfCostFactor(std::vector<ceres::CostFunction*> factors, std::vector<Eigen::Vector3d> normals,
                           const std::vector<point3D>& keypoints, std::shared_ptr<CloudFrame> frame);
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> searchNeighbors(
        VoxelHashMap& map, const Eigen::Vector3d& point_body, int nb_voxel_visited, int size_voxel,
        int num_neighborhoods, int capacity, std::vector<Voxel>* voxels);
    NeightborHood computeNeightborhoodsDistribution(
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& neighborhoods);

   private:
    LioOptions options_;
    double lidar_point_cov = 0.001;

    int index_frame;
    IMUPtr last_imu;

    StaticImuInit static_imu_init;
    double lidar_imu_dt;
    // 从右往左看
    Sophus::SE3d T_IL;
    Eigen::Vector3d P_lidar_in_imu;
    Eigen::Matrix3d R_lidar_to_imu;

    double last_timestamp_imu_;
    double last_timestamp_lidar_;
    double processed_measure_time_;
    std::condition_variable cond;
    std::mutex mtx_data;
    // data
    std::deque<IMUPtr> imu_buff;
    std::deque<std::vector<point3D>> lidar_buffer;
    std::deque<std::pair<double, double>> time_buffer;

    std::vector<std::shared_ptr<State>> all_state;

    MeasureGroup measurement_;
    bool imu_need_init_ = true;
    ESKFD eskf_;
    std::vector<NavState> imu_states_;
    std::shared_ptr<State> current_state;
    // 地图
    VoxelHashMap voxel_map;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world;

    // func
    std::function<bool(std::string& topic_name, CloudPtr& cloud, double time)> pub_cloud_to_ros;
    std::function<bool(std::string& topic_name, Sophus::SE3d& pose, double time)> pub_pose_to_ros;
    std::function<bool(std::string& topic_name, double time1, double time2)> pub_data_to_ros;
};
}  // namespace ctlio