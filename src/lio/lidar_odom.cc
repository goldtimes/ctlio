#include "lio/lidar_odom.hh"

namespace ctlio {
LidarOdom::LidarOdom() {
    //
    // current_state = std::make_shared<State>();
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
    T_IL = Sophus::SE3d(q_IL, t_lidar_in_imu);
    return true;
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
    // std::cout << "push lidar" << std::endl;
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
        lck.unlock();
        // debug
        // if (measurments.size() != 0) {
        //     LOG(INFO) << "meas size:" << measurments.size();
        // }
        // 处理传感器数据
        for (const auto& meas : measurments) {
            Timer::Evaluate([&]() { processMeasurements(meas); }, "processMeasurements");
        }
        // {

        // }
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
        while (!imu_buff.empty() && imu_time < meas.lidar_end_time) {
            imu_time = imu_buff.front()->timestamp_;
            // if (imu_time < meas.lidar_begin_time) {
            //     imu_buff.pop_front();
            //     continue;
            // }
            if (imu_time > meas.lidar_end_time) {
                break;
            }
            meas.imu_datas.push_back(imu_buff.front());
            imu_buff.pop_front();
        }
        // // 多加入一帧 用来做插值
        if (!imu_buff.empty()) {
            meas.imu_datas.push_back(imu_buff.front());
        }
        measure_group.push_back(meas);
    }
}

void LidarOdom::processMeasurements(const MeasureGroup& meas) {
    measurement_ = meas;
    if (imu_need_init_) {
        TryInitIMU();
        return;
    }
    LOG(INFO) << "process frame: " << index_frame;
    // 保存一帧之间的imu积分姿态前，清空之前的数据
    imu_states_.clear();
    // 初始化成功之后，开始predict
    Timer::Evaluate([&]() { Predict(); }, "Predict()");
    // 初始化状态
    // initState
    std::vector<point3D> points_lidar;
    points_lidar.insert(points_lidar.end(), measurement_.lidar_.begin(), measurement_.lidar_.end());
    // scan-to-submap
    std::shared_ptr<CloudFrame> p_frame;
    Timer::Evaluate(
        [&]() {
            p_frame =
                BuildFrame(points_lidar, current_state, measurement_.lidar_begin_time, measurement_.lidar_end_time);
        },
        "build frame");

    // 融合到eskf
    // 可视化
}

// 如何IMU非水平放置呢？
void LidarOdom::TryInitIMU() {
    // 将数据放到static_imu_init中
    // std::cout << "TryInitIMU" << std::endl;
    // LOG(INFO) << "imu size: " << measurement_.imu_datas.size();
    for (const auto& imu : measurement_.imu_datas) {
        static_imu_init.AddImu(*imu);
    }
    // 初始化成功
    if (static_imu_init.InitSuccess()) {
        // // 设置eskf的初始状态
        ESKFD::Options eskf_options;
        eskf_.SetInitialConditions(eskf_options, static_imu_init.GetInitBg(), static_imu_init.GetInitBa(),
                                   static_imu_init.GetGravity());
        imu_need_init_ = false;
    }
}

void LidarOdom::Predict() {
    // 对measgroup_中的所有imu数据去积分，需要保存这帧雷达之间的body姿态信息
    imu_states_.emplace_back(eskf_.GetNominalState());  // 保存开始时刻的姿态
    double lidar_end_time = measurement_.lidar_end_time;
    for (const auto& imu : measurement_.imu_datas) {
        double processed_imu_time = imu->timestamp_;
        if (processed_imu_time <= lidar_end_time) {
            // if (last_imu == nullptr) {
            //     last_imu = imu;
            // }
            eskf_.Predict(*imu);
            imu_states_.emplace_back(eskf_.GetNominalState());
            last_imu = imu;
        } else {
            // 做插值
            double dt_1 = processed_imu_time - lidar_end_time;
            double dt_2 = lidar_end_time - last_imu->timestamp_;
            double w1 = dt_1 / (dt_1 + dt_2);
            double w2 = dt_2 / (dt_1 + dt_2);
            // 加速度插值
            Eigen::Vector3d acc_tmp = w1 * last_imu->acc_ + w2 * imu->acc_;
            Eigen::Vector3d gyro_tmp = w1 * last_imu->gyro_ + w2 * imu->gyro_;
            IMUPtr imu_interp = std::make_shared<IMU>(lidar_end_time, gyro_tmp, acc_tmp);
            eskf_.Predict(*imu_interp);
            imu_states_.emplace_back(eskf_.GetNominalState());
            last_imu = imu_interp;
        }
    }
}

/**
 * @brief 去畸变以及将点云转换到body坐标系下
 */
std::shared_ptr<CloudFrame> LidarOdom::BuildFrame(const std::vector<point3D>& points_lidar,
                                                  std::shared_ptr<State> current_state, double frame_begin_time,
                                                  double frame_end_time) {
    std::vector<point3D> frame(points_lidar);
    if (index_frame < 2) {
        // 第一帧
        for (auto& point : frame) {
            point.alpha_time = 1.0;
        }
    }
    Undistort(frame);
    // 转换到世界系下的点云
    for (auto& point_tmp : frame) {
        transformPoint(MotionCompensation::CONSTANT_VELOCITY, point_tmp, current_state->rotation_begin,
                       current_state->translation_begin, current_state->rotation, current_state->translation,
                       T_IL.rotationMatrix(), T_IL.translation());
    }
    std::shared_ptr<CloudFrame> p_frame = std::make_shared<CloudFrame>(frame, points_lidar, current_state);
    p_frame->dt_offset = 0;
    p_frame->frame_id = index_frame;
    return p_frame;
}

void LidarOdom::Undistort(std::vector<point3D>& points) {
    // 雷达帧的最后时刻姿态
    auto imu_state = eskf_.GetNominalState();
    Sophus::SE3d T_end = Sophus::SE3d(imu_state.R_, imu_state.p_);
    // 将所有点转换到最后时刻的状态
    for (auto& p : points) {
        Sophus::SE3d Ti = T_end;
        NavState match;
        // T_IL * p.point 转换到imu坐标系，然后计算i时刻的坐标，最后转换到末尾时刻，最后转回雷达坐标系
        PoseInterp<NavState>(p.timestamp, imu_states_, [](const NavState& s) { return s.timestamp_; },
                             [](const NavState& s) { return s.GetSE3(); }, Ti, match);
        p.point = T_IL.inverse() * T_end.inverse() * Ti * T_IL * p.point;
    }
}

}  // namespace ctlio