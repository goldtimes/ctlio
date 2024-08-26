#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <random>
#include <thread>
#include "lidar_process/lidar_process.hh"
#include "lio/lidar_odom.hh"
#include "sensors/imu.hh"
#include "sensors/point_types.hh"
#include "tools/lidar_utils.hh"
#include "tools/timer/timer.hh"

DEFINE_string(config_yaml, "./config/ctlio.yaml", "配置文件");
// ROOT_DIR在cmake中定义
#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "log/" + name))

std::string lidar_topic;
std::string imu_topic;
std::shared_ptr<ctlio::LidarProcess> lidar_process;
ros::Publisher pub_scan;
ros::Publisher pub_submap;
ros::Publisher pub_lidar_odom;
ros::Publisher pub_lidar_path;
ros::Subscriber lidar_sub;
ros::Subscriber imu_sub;

nav_msgs::Path lidar_path;

std::shared_ptr<ctlio::LidarOdom> lio;

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // ConstPtr to Ptr;
    sensor_msgs::PointCloud2::Ptr cloud_in(new sensor_msgs::PointCloud2(*msg));
    static int count = 0;
    std::vector<ctlio::point3D> cloud_out;
    ctlio::Timer::Evaluate([&]() { lidar_process->process(cloud_in, cloud_out); }, "lidar process");
    // 利用voxel降采样
    lio->pushLidar(cloud_out, std::make_pair(msg->header.stamp.toSec(), lidar_process->getTimeSpan()));
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    IMUPtr imu = std::make_shared<ctlio::IMU>(msg->header.stamp.toSec(), acc, gyro);
    lio->pushImu(imu);
}

// 这里的nh不能为const
void init_sub_pub(ros::NodeHandle& nh) {
    lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 10, lidar_callback);
    imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 500, imu_callback);
}

int main(int argc, char** argv) {
    // glog
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);
    // ros
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;

    tf::TransformBroadcaster tf_;

    // yaml
    std::string config_file = std::string(ROOT_DIR) + "config/ctlio.yaml";
    std::cout << "ROOT_DIR: " << std::string(ROOT_DIR) << std::endl;
    // std::cout << "yaml_path: " << yaml_path << std::endl;
    YAML::Node yaml = YAML::LoadFile(config_file);
    lidar_topic = yaml["common"]["lidar_topic"].as<std::string>();
    imu_topic = yaml["common"]["imu_topic"].as<std::string>();
    std::cout << "lidar_topic: " << lidar_topic << std::endl;
    std::cout << "imu_topic: " << imu_topic << std::endl;
    // lio
    lio = std::make_shared<ctlio::LidarOdom>();
    if (!lio->init(config_file)) {
        return -1;
    }

    pub_scan = nh.advertise<sensor_msgs::PointCloud2>("laser", 10);
    pub_submap = nh.advertise<sensor_msgs::PointCloud2>("submap", 10);
    pub_lidar_odom = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    pub_lidar_path = nh.advertise<nav_msgs::Path>("/lidar_path", 10);

    auto cloud_pub_func = std::function<bool(std::string & topic_name, ctlio::CloudPtr & cloud, double time)>(
        [&](std::string& topic_name, ctlio::CloudPtr& cloud, double time) {
            sensor_msgs::PointCloud2::Ptr cloud_ros_output(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*cloud, *cloud_ros_output);
            cloud_ros_output->header.stamp = ros::Time::now();
            cloud_ros_output->header.frame_id = "map";
            pub_scan.publish(*cloud_ros_output);
            return true;
        });

    auto submap_pub_func =
        std::function<bool(ctlio::CloudPtr & cloud, double time)>([&](ctlio::CloudPtr& cloud, double time) {
            sensor_msgs::PointCloud2::Ptr cloud_ros_output(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*cloud, *cloud_ros_output);
            cloud_ros_output->header.stamp = ros::Time::now();
            cloud_ros_output->header.frame_id = "map";
            pub_submap.publish(*cloud_ros_output);
            return true;
        });

    auto pose_path_pub_func = std::function<bool(std::string & topic_name, Sophus::SE3d & pose, double timestamp)>(
        [&](std::string& topic_name, Sophus::SE3d& pose, double timestamp) {
            tf::Transform transform;
            Eigen::Quaterniond q_eigen(pose.so3().matrix());
            transform.setOrigin(tf::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
            tf::Quaternion q(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
            transform.setRotation(q);
            if (topic_name == "laser") {
                // 发布tf
                tf_.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(timestamp), "map", "base_link"));
                // odom
                nav_msgs::Odometry laserOdometry;
                laserOdometry.header.frame_id = "map";
                laserOdometry.child_frame_id = "base_link";
                laserOdometry.header.stamp = ros::Time().fromSec(timestamp);
                laserOdometry.pose.pose.position.x = pose.translation().x();
                laserOdometry.pose.pose.position.y = pose.translation().y();
                laserOdometry.pose.pose.position.z = pose.translation().z();

                laserOdometry.pose.pose.orientation.x = q_eigen.x();
                laserOdometry.pose.pose.orientation.y = q_eigen.y();
                laserOdometry.pose.pose.orientation.z = q_eigen.z();
                laserOdometry.pose.pose.orientation.w = q_eigen.w();

                pub_lidar_odom.publish(laserOdometry);

                geometry_msgs::PoseStamped laser_pose;
                laser_pose.header = laserOdometry.header;
                laser_pose.pose = laserOdometry.pose.pose;
                lidar_path.header.stamp = laserOdometry.header.stamp;
                lidar_path.poses.push_back(laser_pose);
                lidar_path.header.frame_id = "map";
                pub_lidar_path.publish(lidar_path);
            }

            return true;
        });
    ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("velocity", 1);
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float32>("move_dist", 1);

    auto data_pub_func = std::function<bool(std::string & topic_name, double time1, double time2)>(
        [&](std::string& topic_name, double time1, double time2) {
            std_msgs::Float32 time_rviz;
            time_rviz.data = time1;
            if (topic_name == "velocity") {
                vel_pub.publish(time_rviz);
            } else {
                dist_pub.publish(time_rviz);
            }

            return true;
        });
    lio->setFunc(cloud_pub_func);
    lio->setFunc(pose_path_pub_func);
    lio->setFunc(data_pub_func);
    // lio->setFunc(submap_pub_func);

    // lidar_process
    lidar_process = std::make_shared<ctlio::LidarProcess>();
    lidar_process->LoadYaml(config_file);
    init_sub_pub(nh);

    std::thread lio_thread(&ctlio::LidarOdom::run, lio);

    ros::spin();
    return 0;
}