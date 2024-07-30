#include "lio/lidar_odom.hh"

namespace ctlio {
LidarOdom::LidarOdom() {
    //
}

bool LidarOdom::init(const std::string& config_file) {
    StaticImuInit::InitOptions init_options;
    init_options.use_odom = false;
    static_imu_init = StaticImuInit(init_options);

    auto yaml = YAML::LoadFile(config_file);
    loadOptions(config_file);
    lidar_imu_dt = yaml["lio"]["delay_time"].as<float>();
    std::vector<double> extr_t = yaml["lio"]["extrinsic_T"].as<std::vector<double>>();
    std::vector<double> extr_r = yaml["lio"]["extrinsic_R"].as<std::vector<double>>();
    Eigen::Vector3d t_lidar_in_imu(extr_t[0], extr_t[1], extr_t[2]);
    Eigen::Matrix3d r_lidar_in_imu;
    // clang-format off
    r_lidar_in_imu << extr_r[0], extr_r[1], extr_r[2],
                      extr_r[3], extr_r[4], extr_r[5], 
                      extr_r[6], extr_r[7], extr_r[8];
    // clang-format on
    Eigen::Quaterniond q_IL(r_lidar_in_imu);
    q_IL.normalize();
    P_lidar_in_imu = t_lidar_in_imu;
    R_lidar_to_imu = r_lidar_in_imu;
    T_IL = Sophus::SE3(q_IL, t_lidar_in_imu);
}

void LidarOdom::loadOptions(const std::string& config_file) {
    auto yaml = YAML::LoadFile(config_file);
    // options;
}

void LidarOdom::pushImu(IMUPtr& imu_msg) {
    double time = imu_msg->timestamp_;
    if (time < last_timestamp_imu_) {
        LOG(INFO) << "imu loop back";
        imu_buff.clear();
    }
    last_timestamp_imu_ = time;
    std::lock_guard<std::mutex> lck(mtx_data);
    imu_buff.emplace_back(imu_msg);
    cond.notify_one();
}

void LidarOdom::pushLidar(const std::vector<point3D>& msg, std::pair<double, double> data) {
    if (data.first < last_timestamp_lidar_) {
        LOG(ERROR) << "lidar loop back";
        lidar_buffer.clear();
        time_buffer.clear();
    }
    std::lock_guard<std::mutex> lck(mtx_data);
    time_buffer.push_back(data);
    lidar_buffer.push_back(msg);
    last_timestamp_lidar_ = data.first;
    cond.notify_one();
}

void LidarOdom::run() {
    while (ros::ok()) {
        // 时间同步
        std::vector<MeasureGroup> measurments;
        std::unique_lock<std::mutex> lck(mtx_data);
        cond.wait(lck, [&]() -> bool {
            measurments = getMeasurments();
            return measurments.size() != 0;
        });
        // debug
        if (measurments.size() != 0) {
            LOG(INFO) << "meas size:" << measurments.size();
        }
        // 处理传感器数据
    }
}
std::vector<MeasureGroup> LidarOdom::getMeasurments() {
    std::vector<MeasureGroup> measure_group;
    while (true) {
        if (imu_buff.empty()) {
            return measure_group;
        }
        if (lidar_buffer.empty()) {
            return measure_group;
        }

        MeasureGroup meas;
        meas.lidar_ = lidar_buffer.front();
        meas.lidar_begin_time = time_buffer.front().first;
        meas.lidar_end_time = meas.lidar_begin_time + time_buffer.front().second;
        lidar_buffer.pop_front();
        time_buffer.pop_front();
        // 记录雷达末尾时间戳
        processed_measure_time_ = meas.lidar_end_time;
        double imu_time = imu_buff.front()->timestamp_;
        meas.imu_datas.clear();
        while (!imu_buff.empty()) {
            if (imu_time < meas.lidar_begin_time) {
                imu_buff.pop_front();
                imu_time = imu_buff.front()->timestamp_;
                continue;
            }
            if (imu_time > meas.lidar_end_time) {
                break;
            }
            meas.imu_datas.push_back(imu_buff.front());
            imu_buff.pop_front();
        }
        // 多加入一帧 用来做插值
        if (!imu_buff.empty()) {
            meas.imu_datas.push_back(imu_buff.front());
        }
        measure_group.push_back(meas);
    }
}

}  // namespace ctlio