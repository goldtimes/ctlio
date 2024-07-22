#include "lidar_process/lidar_process.hh"

namespace ctlio {
void LidarProcess::LoadYaml(const std::string& yaml_file) {
    YAML::Node yaml = YAML::LoadFile(yaml_file);
    int type = yaml["preprocess"]["lidar_type"].as<int>();
    // point filter num
    point_filter_num = yaml["preprocess"]["point_filter_num"].as<int>();
    // blind
    blind = yaml["preprocess"]["blind"].as<float>();
    std::cout << "lidar_type:" << type << std::endl;
    std::cout << "point_filter_num:" << point_filter_num << std::endl;
    std::cout << "blind:" << blind << std::endl;
    if (type == 1) {
        lidar_type = LidarType::AVIA;
    } else if (type == 2) {
        lidar_type = LidarType::VELO32;
    } else if (type == 3) {
        lidar_type = LidarType::OUST64;
    } else if (type == 4) {
        lidar_type = LidarType::ROBOSENS16;
    } else if (type == 5) {
        lidar_type = LidarType::PANDAR;
    } else if (type == 6) {
        lidar_type = LidarType::LS;
    }
}

void LidarProcess::process(const livox_ros_driver::CustomMsg::Ptr& msg, std::vector<point3D>& pcl_out) {
    AviaHandler(msg, pcl_out);
}

void LidarProcess::process(const sensor_msgs::PointCloud2::Ptr& msg, std::vector<point3D>& pcl_out) {
    switch (lidar_type) {
        case LidarType::VELO32:
            VelodyneHandler(msg, pcl_out);
            break;
        case LidarType::OUST64:
            OusterHandler(msg, pcl_out);
            break;
        case LidarType::ROBOSENS16:
            RobosenseHandler(msg, pcl_out);
            break;
        case LidarType::PANDAR:
            PandarHandler(msg, pcl_out);
            break;
        case LidarType::LS:
            LslidarHandler(msg, pcl_out);
            break;
        default:
            break;
    }
}

void LidarProcess::AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr& msg, std::vector<point3D>& pcl_out) {
    int point_nums = msg->point_num;
    std::vector<point3D> cloud_out;
    pcl_out.clear();
    pcl_out.reserve(point_nums);
    double header_time = msg->header.stamp.toSec();
    // msg中点的时间戳是ns
    timespan_ = static_cast<double>(msg->points.back().offset_time) / 1e9;
    for (int i = 0; i < point_nums; ++i) {
        // 如果不是有限浮点数
        if (!(std::isfinite(msg->points[i].x) && std::isfinite(msg->points[i].y) && std::isfinite(msg->points[i].z))) {
            continue;
        }
        // 过滤点
        if (i % point_filter_num != 0) {
            continue;
        }
        double range = msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y +
                       msg->points[i].z * msg->points[i].z;
        // 距离过滤
        if (range > 150 * 150 || range < blind * blind) {
            continue;
        }
        // 正确的点
        if (((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
            point3D p3d;
            p3d.point = Eigen::Vector3d(msg->points[i].x, msg->points[i].y, msg->points[i].z);
            p3d.point_world = p3d.point;
            p3d.intensity = msg->points[i].reflectivity;
            p3d.timespan = timespan_;
            p3d.ring = msg->points[i].line;
            // 参考当前帧的时间,就是以当前帧的起始为0
            p3d.relative_time = msg->points[i].offset_time / 1e9;
            // 每个点的时间戳
            p3d.timestamp = header_time + p3d.relative_time;
            // relative_time / 0.1
            p3d.alpha_time = p3d.relative_time / timespan_;
            pcl_out.push_back(p3d);
        }
    }
}
void LidarProcess::VelodyneHandler(const sensor_msgs::PointCloud2::Ptr& msg, std::vector<point3D>& pcl_out) {
    pcl_out.clear();
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int point_nums = pl_orig.points.size();
    pcl_out.reserve(point_nums);
    double header_time = msg->header.stamp.toSec();
    static double time_scale = 1.0;
    // velodyne的特殊处理
    std::sort(pl_orig.points.begin(), pl_orig.points.end(),
              [&](const velodyne_ros::Point& point_1, const velodyne_ros::Point& point_2) -> bool {
                  return point_1.time < point_2.time;
              });
    while (pl_orig.points[point_nums - 1].time / time_scale >= 0.1) {
        point_nums--;
        pl_orig.points.pop_back();
    }
    // 计算一帧的时间
    timespan_ = pl_orig.points.back().time / time_scale;

    for (int i = 0; i < point_nums; ++i) {
        // 如果不是有限浮点数
        if (!(std::isfinite(pl_orig.points[i].x) && std::isfinite(pl_orig.points[i].y) &&
              std::isfinite(pl_orig.points[i].z))) {
            continue;
        }
        // 过滤点
        if (i % point_filter_num != 0) {
            continue;
        }

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;
        // 距离过滤
        if (range > 150 * 150 || range < blind * blind) {
            continue;
        }
        point3D p3d;
        p3d.point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
        p3d.point_world = p3d.point;
        p3d.intensity = pl_orig.points[i].intensity;
        p3d.ring = pl_orig.points[i].ring;
        p3d.timespan = timespan_;
        p3d.relative_time = pl_orig.points[i].time / time_scale;
        p3d.timestamp = header_time + p3d.relative_time;
        p3d.alpha_time = p3d.relative_time / timespan_;
        pcl_out.push_back(p3d);
    }
}
void LidarProcess::OusterHandler(const sensor_msgs::PointCloud2::Ptr& msg, std::vector<point3D>& pcl_out) {
    pcl_out.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int point_nums = pl_orig.points.size();
    double header_time = msg->header.stamp.toSec();
    timespan_ = pl_orig.points.back().t / 1e9;
    pcl_out.reserve(point_nums);
    for (int i = 0; i < point_nums; ++i) {
        // 如果不是有限浮点数
        if (!(std::isfinite(pl_orig.points[i].x) && std::isfinite(pl_orig.points[i].y) &&
              std::isfinite(pl_orig.points[i].z))) {
            continue;
        }
        // 过滤点
        if (i % point_filter_num != 0) {
            continue;
        }

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;
        // 距离过滤
        if (range > 150 * 150 || range < blind * blind) {
            continue;
        }

        point3D p3d;
        p3d.point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
        p3d.point_world = p3d.point;
        p3d.intensity = pl_orig.points[i].intensity;
        p3d.ring = pl_orig.points[i].ring;
        p3d.timespan = timespan_;
        p3d.relative_time = pl_orig.points[i].t / 1e9;
        p3d.timestamp = header_time + p3d.relative_time;
        p3d.alpha_time = p3d.relative_time / timespan_;
        pcl_out.push_back(p3d);
    }
}
void LidarProcess::PandarHandler(const sensor_msgs::PointCloud2::Ptr& msg, std::vector<point3D>& pcl_out) {
    pcl_out.clear();
    pcl::PointCloud<pandar_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int point_nums = pl_orig.points.size();
    pcl_out.reserve(point_nums);
    double header_time = msg->header.stamp.toSec();
    static double time_scale = 1;
    // sort
    std::sort(pl_orig.points.begin(), pl_orig.points.end(),
              [&](const pandar_ros::Point& point_1, const pandar_ros::Point& point_2) -> bool {
                  return point_1.timestamp < point_2.timestamp;
              });
    while (pl_orig.points[point_nums - 1].timestamp - pl_orig.points[0].timestamp >= 0.1) {
        point_nums--;
        pl_orig.points.pop_back();
    }
    timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;
    for (int i = 0; i < point_nums; ++i) {
        // 如果不是有限浮点数
        if (!(std::isfinite(pl_orig.points[i].x) && std::isfinite(pl_orig.points[i].y) &&
              std::isfinite(pl_orig.points[i].z))) {
            continue;
        }
        // 过滤点
        if (i % point_filter_num != 0) {
            continue;
        }

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;
        // 距离过滤
        if (range > 150 * 150 || range < blind * blind) {
            continue;
        }

        point3D p3d;
        p3d.point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
        p3d.point_world = p3d.point;
        p3d.intensity = pl_orig.points[i].intensity;
        p3d.ring = pl_orig.points[i].ring;
        p3d.timespan = timespan_;
        p3d.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp;
        p3d.timestamp = header_time + p3d.relative_time;
        p3d.alpha_time = p3d.relative_time / timespan_;
        pcl_out.push_back(p3d);
    }
}
void LidarProcess::RobosenseHandler(const sensor_msgs::PointCloud2::Ptr& msg, std::vector<point3D>& pcl_out) {
    pcl_out.clear();
    pcl::PointCloud<robosense_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int point_nums = pl_orig.points.size();
    pcl_out.reserve(point_nums);
    double header_time = msg->header.stamp.toSec();
    static double time_scale = 1;
    // sort
    std::sort(pl_orig.points.begin(), pl_orig.points.end(),
              [&](const robosense_ros::Point& point_1, const robosense_ros::Point& point_2) -> bool {
                  return point_1.timestamp < point_2.timestamp;
              });
    while (pl_orig.points[point_nums - 1].timestamp - pl_orig.points[0].timestamp >= 0.1) {
        point_nums--;
        pl_orig.points.pop_back();
    }
    timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;
    for (int i = 0; i < point_nums; ++i) {
        // 如果不是有限浮点数
        if (!(std::isfinite(pl_orig.points[i].x) && std::isfinite(pl_orig.points[i].y) &&
              std::isfinite(pl_orig.points[i].z))) {
            continue;
        }
        // 过滤点
        if (i % point_filter_num != 0) {
            continue;
        }

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;
        // 距离过滤
        if (range > 150 * 150 || range < blind * blind) {
            continue;
        }

        point3D p3d;
        p3d.point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
        p3d.point_world = p3d.point;
        p3d.intensity = pl_orig.points[i].intensity;
        p3d.ring = pl_orig.points[i].ring;
        p3d.timespan = timespan_;
        p3d.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp;
        p3d.timestamp = header_time + p3d.relative_time;
        p3d.alpha_time = p3d.relative_time / timespan_;
        pcl_out.push_back(p3d);
    }
}
void LidarProcess::LslidarHandler(const sensor_msgs::PointCloud2::Ptr& msg, std::vector<point3D>& pcl_out) {
    pcl_out.clear();
    pcl::PointCloud<lslidar_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int point_nums = pl_orig.points.size();
    pcl_out.reserve(point_nums);
    double header_time = msg->header.stamp.toSec();
    static double time_scale = 1;
    // sort
    std::sort(pl_orig.points.begin(), pl_orig.points.end(),
              [&](const lslidar_ros::Point& point_1, const lslidar_ros::Point& point_2) -> bool {
                  return point_1.time < point_2.time;
              });
    while (pl_orig.points[point_nums - 1].time - pl_orig.points[0].time >= 0.1) {
        point_nums--;
        pl_orig.points.pop_back();
    }
    timespan_ = pl_orig.points.back().time - pl_orig.points[0].time;
    for (int i = 0; i < point_nums; ++i) {
        // 如果不是有限浮点数
        if (!(std::isfinite(pl_orig.points[i].x) && std::isfinite(pl_orig.points[i].y) &&
              std::isfinite(pl_orig.points[i].z))) {
            continue;
        }
        // 过滤点
        if (i % point_filter_num != 0) {
            continue;
        }

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;
        // 距离过滤
        if (range > 150 * 150 || range < blind * blind) {
            continue;
        }

        point3D p3d;
        p3d.point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
        p3d.point_world = p3d.point;
        p3d.intensity = pl_orig.points[i].intensity;
        p3d.ring = pl_orig.points[i].ring;
        p3d.timespan = timespan_;
        p3d.relative_time = pl_orig.points[i].time - pl_orig.points[0].time;
        p3d.timestamp = header_time + p3d.relative_time;
        p3d.alpha_time = p3d.relative_time / timespan_;
        pcl_out.push_back(p3d);
    }
}

}  // namespace ctlio