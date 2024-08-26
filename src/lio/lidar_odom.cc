#include "lio/lidar_odom.hh"

namespace ctlio {
LidarOdom::LidarOdom() {
    lidar_point_cov = 0.001;
    current_state = std::make_shared<State>(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
                                            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    LidarPlaneNormalFactor::sqrt_info = std::sqrt(1 / lidar_point_cov);
    index_frame = 1;
    cloud_world.reset(new pcl::PointCloud<pcl::PointXYZI>());
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
    std::cout << "t_IL:\n" << t_lidar_in_imu << std::endl;
    std::cout << "r_IL:\n" << r_lidar_in_imu << std::endl;
    Eigen::Quaterniond q_IL(r_lidar_in_imu);
    q_IL.normalize();
    P_lidar_in_imu = t_lidar_in_imu;
    R_lidar_to_imu = r_lidar_in_imu;
    T_IL = Sophus::SE3d(q_IL, t_lidar_in_imu);
    LidarPlaneNormalFactor::q_IL = T_IL.rotationMatrix();
    LidarPlaneNormalFactor::t_IL = P_lidar_in_imu;
    return true;
}

void LidarOdom::loadOptions(const std::string& config_file) {
    auto yaml = YAML::LoadFile(config_file);
    // options;
    options_.surf_res = yaml["odometry"]["surf_res"].as<double>();
    options_.max_num_iteration = yaml["odometry"]["max_num_iteration"].as<int>();

    options_.size_voxel_map = yaml["odometry"]["size_voxel_map"].as<double>();
    options_.min_distance_points = yaml["odometry"]["min_distance_points"].as<double>();
    options_.max_num_points_in_voxel = yaml["odometry"]["max_num_points_in_voxel"].as<int>();
    options_.max_distance = yaml["odometry"]["max_distance"].as<double>();
    options_.weight_alpha = yaml["odometry"]["weight_alpha"].as<double>();
    options_.weight_neighborhood = yaml["odometry"]["weight_neighborhood"].as<double>();
    options_.max_dist_to_plane_icp = yaml["odometry"]["max_dist_to_plane_icp"].as<double>();
    options_.init_num_frames = yaml["odometry"]["init_num_frames"].as<int>();
    options_.voxel_neighborhood = yaml["odometry"]["voxel_neighborhood"].as<int>();
    options_.max_number_neighbors = yaml["odometry"]["max_number_neighbors"].as<int>();
    options_.threshold_voxel_occupancy = yaml["odometry"]["threshold_voxel_occupancy"].as<int>();
    options_.estimate_normal_from_neighborhood = yaml["odometry"]["estimate_normal_from_neighborhood"].as<bool>();
    options_.min_number_neighbors = yaml["odometry"]["min_number_neighbors"].as<int>();
    options_.power_planarity = yaml["odometry"]["power_planarity"].as<double>();
    options_.num_closest_neighbors = yaml["odometry"]["num_closest_neighbors"].as<int>();

    options_.sampling_rate = yaml["odometry"]["sampling_rate"].as<double>();
    // options_.ratio_of_nonground = yaml["odometry"]["ratio_of_nonground"].as<double>();
    options_.max_num_residuals = yaml["odometry"]["max_num_residuals"].as<int>();
    std::string str_motion_compensation = yaml["odometry"]["motion_compensation"].as<std::string>();
    if (str_motion_compensation == "NONE")
        options_.motion_compensation = MotionCompensation::NONE;
    else if (str_motion_compensation == "CONSTANT_VELOCITY")
        options_.motion_compensation = MotionCompensation::CONSTANT_VELOCITY;
    else if (str_motion_compensation == "ITERATIVE")
        options_.motion_compensation = MotionCompensation::ITERATIVE;
    else if (str_motion_compensation == "CONTINUOUS")
        options_.motion_compensation = MotionCompensation::CONTINUOUS;
    else
        std::cout << "The `motion_compensation` " << str_motion_compensation << " is not supported." << std::endl;

    std::string str_icpmodel = yaml["odometry"]["icpmodel"].as<std::string>();
    if (str_icpmodel == "POINT_TO_PLANE")
        options_.icp_model = POINT_TO_PLANE;
    else if (str_icpmodel == "CT_POINT_TO_PLANE")
        options_.icp_model = CT_POINT_TO_PLANE;
    else
        std::cout << "The `icp_residual` " << str_icpmodel << " is not supported." << std::endl;

    // options_.beta_location_consistency = yaml["odometry"]["beta_location_consistency"].as<double>();
    // options_.beta_orientation_consistency = yaml["odometry"]["beta_orientation_consistency"].as<double>();
    // options_.beta_constant_velocity = yaml["odometry"]["beta_constant_velocity"].as<double>();
    // options_.beta_small_velocity = yaml["odometry"]["beta_small_velocity"].as<double>();
    options_.thres_rotation_norm = yaml["odometry"]["thres_rotation_norm"].as<double>();
    options_.thres_trans_norm = yaml["odometry"]["thres_trans_norm"].as<double>();
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
        for (auto& meas : measurments) {
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

void LidarOdom::processMeasurements(MeasureGroup& meas) {
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
    Timer::Evaluate([&]() { stateInitialization(); }, "state init");
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
    Timer::Evaluate([&]() { PoseEstimation(p_frame); }, "pose estimate");
    Sophus::SE3d pose_of_lo = Sophus::SE3d(current_state->rotation, current_state->translation);
    // 更新ekf
    Timer::Evaluate([&]() { eskf_.ObserveSE3(pose_of_lo, 1e-2, 1e-2); }, "update eskf obs");
    // 可视化
    Timer::Evaluate(
        [&]() {
            std::string topic_name = "laser";
            pub_pose_to_ros(topic_name, pose_of_lo, meas.lidar_end_time);
            // pub_cloud_to_ros(topic_name, )
        },
        "vis");
    p_frame->p_state = std::make_shared<State>(current_state, true);
    std::shared_ptr<State> tmp_state = std::make_shared<State>(current_state, true);
    all_state.push_back(tmp_state);
    current_state = std::make_shared<State>(current_state, false);
    index_frame++;
    p_frame->release();
    meas.lidar_.clear();
    points_lidar.clear();
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

void LidarOdom::stateInitialization() {
    if (index_frame < 2) {
        current_state->rotation_begin = Eigen::Quaterniond(imu_states_.front().R_.matrix());
        current_state->translation_begin = imu_states_.front().p_;
        current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
        current_state->translation = imu_states_.back().p_;
    } else {
        current_state->rotation_begin = all_state[all_state.size() - 1]->rotation;
        current_state->translation_begin = all_state[all_state.size() - 1]->translation;
        current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
        current_state->translation = imu_states_.back().p_;
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
                       current_state->rotation, current_state->translation_begin, current_state->translation,
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
void LidarOdom::PoseEstimation(std::shared_ptr<CloudFrame> frame) {
    // 第一帧直接跳过
    if (index_frame > 1) {
    }
    // 添加点到voxelmap中
    Timer::Evaluate([&]() { map_incremental(frame); }, "update map");
    // 删除过远的体素
    Timer::Evaluate([&]() { lasermap_fov_segment(); }, "fov segments");
}

void LidarOdom::map_incremental(std::shared_ptr<CloudFrame> frame) {
    // 将cloud frame中的世界坐标系下的地图点放到voxelmap中
    for (const auto& point3D_tmp : frame->points_world) {
        int min_num_points = 0;
        AddPointToMap(voxel_map, point3D_tmp.point_world, point3D_tmp.intensity, options_.size_voxel_map,
                      options_.max_num_points_in_voxel, options_.min_distance_points, 0);
    }
    // vis
    std::string laser_topic = "laser";
    pub_cloud_to_ros(laser_topic, cloud_world, frame->time_frame_end);
    // 清空该帧点云
    cloud_world->clear();
}
void LidarOdom::lasermap_fov_segment() {
    Eigen::Vector3d laser_location = current_state->translation;
    std::vector<Voxel> voxels_to_erase;
    for (auto& pair : voxel_map) {
        // 取出第一个点
        Eigen::Vector3d pt = pair.second.points[0];
        if ((pt - laser_location).squaredNorm() > (options_.max_distance * options_.max_distance)) {
            voxels_to_erase.push_back(pair.first);
        }
    }
    for (auto& voxel : voxels_to_erase) {
        voxel_map.erase(voxel);
    }
    voxels_to_erase.clear();
}

void LidarOdom::AddPointToMap(VoxelHashMap& map, const Eigen::Vector3d& point, const double intensity,
                              double voxel_size, int max_num_in_voxel, double min_distance_points, int min_num_points) {
    // 体素化
    short coord_x = static_cast<short>(point.x() / voxel_size);
    short coord_y = static_cast<short>(point.y() / voxel_size);
    short coord_z = static_cast<short>(point.z() / voxel_size);
    Voxel voxel_grid(coord_x, coord_y, coord_z);
    VoxelHashMap::iterator iter = map.find(voxel_grid);
    if (iter != map.end()) {
        // 获取voxelblcok
        auto& voxel_block = iter.value();
        // 体素的点云没有达到最大值
        if (!voxel_block.IsFull()) {
            // 记录体素中的点到point的最小距离
            double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
            // 计算体素中的点到该点的距离平方
            for (int i = 0; i < voxel_block.GetNumPoints(); ++i) {
                auto& voxel_point = voxel_block.points[i];
                double sq_dist = (voxel_point - point).squaredNorm();
                if (sq_dist < sq_dist_min_to_points) {
                    sq_dist_min_to_points = sq_dist;
                }
            }
            if (sq_dist_min_to_points > (min_distance_points * min_distance_points)) {
                if (min_num_points <= 0 || voxel_block.GetNumPoints() >= min_num_points) {
                    voxel_block.AddPoint(point);
                }
            }
        }
    } else {
        // 创建voxel_block
        if (min_num_points <= 0) {
            VoxelBlock block(max_num_in_voxel);
            block.AddPoint(point);
            map[voxel_grid] = std::move(block);
        }
    }
    // add point to cloud world
    AddPointToCloud(cloud_world, point, intensity);
}

void LidarOdom::AddPointToCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const Eigen::Vector3d& point,
                                const double intensity) {
    pcl::PointXYZI cloud_point;
    cloud_point.x = point.x();
    cloud_point.y = point.y();
    cloud_point.z = point.z();
    cloud_point.intensity = intensity;
    cloud->push_back(cloud_point);
}

void LidarOdom::Optimize(std::shared_ptr<CloudFrame> frame) {
    std::shared_ptr<State> prev_state;
    Eigen::Vector3d prev_translation = Eigen::Vector3d::Zero();
    Eigen::Vector3d prev_velocity = Eigen::Vector3d::Zero();
    Eigen::Quaterniond prev_orientation = Eigen::Quaterniond::Identity();

    std::shared_ptr<State> curr_state = frame->p_state;
    Eigen::Quaterniond begin_quat = Eigen::Quaterniond(curr_state->rotation_begin);
    Eigen::Quaterniond end_quat = Eigen::Quaterniond(curr_state->rotation);
    Eigen::Vector3d begin_t = curr_state->translation_begin;
    Eigen::Vector3d end_t = curr_state->translation;
    if (frame->frame_id > 1) {
        prev_state = all_state[frame->frame_id - 2];
        prev_translation = prev_state->translation;
        prev_velocity = prev_state->translation - prev_state->translation_begin;
        prev_orientation = prev_state->rotation;
    }
    // 降采样
    std::vector<point3D> keypoints;
    GridSampling(frame->points_world, keypoints, options_.sampling_rate * options_.surf_res);
    size_t num_size = frame->points_world.size();

    auto transformKeypoints = [&](std::vector<point3D>& point_frame) {
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        for (auto& keypoint : point_frame) {
            if (options_.point_to_plane_with_distortion || options_.icp_model == ICPMODEL::CT_POINT_TO_PLANE) {
            } else {
                R = end_quat.normalized().toRotationMatrix();
                t = end_t;
            }
            keypoint.point_world = R * (T_IL * keypoint.point) + t;
        }
    };
    for (int iter = 0; iter < options_.max_num_iteration; iter++) {
        transformKeypoints(keypoints);
        ceres::LossFunction* loss_function = new ceres::HuberLoss(0.5);
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        // 默认使用point_to_plane
        // 添加待优化的变量(位姿)
        ceres::LocalParameterization* parameterization = new RotationParameterization();
        // auto* parameterization = new ceres::EigenQuaternionManifold();
        // 四元素代表的旋转
        problem.AddParameterBlock(&end_quat.x(), 4, parameterization);
        // translation
        problem.AddParameterBlock(&end_t.x(), 3);
        // 构造点面的cost_function
        std::vector<ceres::CostFunction*> point_to_plane_factors;
        std::vector<Eigen::Vector3d> normal_vecs;
        AddSurfCostFactor(point_to_plane_factors, normal_vecs, keypoints, frame);
        // checkLocalizability;
        // 遍历所有的残差快
        int factor_num = 0;
        for (const auto& error : point_to_plane_factors) {
            factor_num++;
            problem.AddResidualBlock(error, loss_function, &end_t.x(), &end_quat.x());
        }
        normal_vecs.clear();
        point_to_plane_factors.clear();
        // 求解
        if (factor_num < options_.min_num_resdiuals) {
            std::stringstream ss;
            ss << "[Optimization] Error : not enough keypoints select in icp!" << std::endl;
            ss << "[Optimization] number_of_residuals : " << factor_num << std::endl;
            std::cout << ss.str();
        }
        ceres::Solver::Options options;
        options.max_num_iterations = 5;
        options.num_threads = 3;
        options.minimizer_progress_to_stdout = false;
        options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
        // 关键帧
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        // usable 可用
        if (!summary.IsSolutionUsable()) {
            std::cout << summary.FullReport() << std::endl;
            throw std::runtime_error("error during optimiaztion");
        }
        // 更新state
        begin_quat.normalize();
        end_quat.normalize();

        double diff_trans = 0;
        double diff_rot = 0;
        diff_trans += (curr_state->translation_begin - begin_t).norm();
        diff_rot += AngularDistance(curr_state->rotation_begin, begin_quat);

        diff_trans += (curr_state->translation - end_t).norm();
        diff_rot += AngularDistance(curr_state->rotation, end_quat);

        frame->p_state->translation = end_t;
        frame->p_state->rotation = end_quat;

        curr_state->translation = end_t;
        curr_state->rotation = end_quat;
        if (diff_rot < options_.thres_rotation_norm && diff_trans < options_.thres_trans_norm) {
            std::cout << "Finished with N=" << iter << " ICP iterations" << std::endl;
            break;
        }
    }
    keypoints.clear();
    std::cout << "opt beign:" << frame->p_state->translation_begin.transpose() << ",end"
              << frame->p_state->translation.transpose() << std::endl;
    transformKeypoints(frame->points_world);
}

void LidarOdom::AddSurfCostFactor(std::vector<ceres::CostFunction*> factors, std::vector<Eigen::Vector3d> normals,
                                  const std::vector<point3D>& keypoints, std::shared_ptr<CloudFrame> frame) {
    auto estimatePointNeightborhood =
        [&](std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& neighborhoods,
            Eigen::Vector3d& point_body, double& weight) -> Neighborhood {
        Neighborhood neightbor = computeNeightborhoodsDistribution(neighborhoods);
        // planarity 平面性
        weight = std::pow(neightbor.a2D, options_.power_planarity);
        if (neightbor.normal.dot(frame->p_state->translation_begin - point_body) < 0) {
            neightbor.normal = -1.0 * neightbor.normal;
        }

        return neightbor;
    };

    double lamda_weight = std::abs(options_.weight_alpha);
    double lamda_neighborhood = std::abs(options_.weight_neighborhood);

    double kMaxDisPointToPlan = options_.max_dist_to_plane_icp;

    double sum = lamda_weight + lamda_neighborhood;
    lamda_weight /= sum;
    lamda_neighborhood /= sum;
    int nb_voxels_visited = frame->frame_id < options_.init_num_frames ? 2 : options_.voxel_neighborhood;
    // 临近的体素中，点的阈值。小于该阈值的体素不需要
    int kThresholdCapacity = frame->frame_id < options_.init_num_frames ? 1 : options_.threshold_voxel_occupancy;

    int num = keypoints.size();
    int num_residuals = 0;
    // 接下来就是为每个点构造ceres的factor
    for (int k = 0; k < num; k++) {
        auto& keypoint = keypoints[k];
        auto point_lidar = keypoint.point;
        auto point_world = keypoint.point_world;
        std::vector<Voxel> voxels;
        // 得到周围最近的点
        auto vector_neighbors = searchNeighbors(voxel_map, point_world, nb_voxels_visited, options_.size_voxel_map,
                                                options_.max_number_neighbors, kThresholdCapacity,
                                                options_.estimate_normal_from_neighborhood ? nullptr : &voxels);
        if (vector_neighbors.size() < options_.min_number_neighbors) {
            continue;
        }
        double weight;
        Eigen::Vector3d point_body = T_IL * point_lidar;
        // 将最近的点进行主成份分析,得到这些点的重心，协方差以及特征向量(衡量是否为平面)
        auto neighborhood = estimatePointNeightborhood(vector_neighbors, point_body, weight);
        weight =
            lamda_weight * weight + lamda_neighborhood * std::exp(-(vector_neighbors[0] - point_world).norm() /
                                                                  (kMaxDisPointToPlan * options_.min_number_neighbors));
        double point_to_plane_dist;
        std::vector<Voxel> neighbor_voxels;
        for (int i = 0; i < options_.num_closest_neighbors; ++i) {
            point_to_plane_dist = std::abs((point_world - vector_neighbors[i]).transpose() * neighborhood.normal);
            // 距离合适
            if (point_to_plane_dist < options_.max_dist_to_plane_icp) {
                num_residuals++;
                Eigen::Vector3d norm_vector = neighborhood.normal;
                norm_vector.normalized();
                normals.push_back(norm_vector);
                double norm_offset = -norm_vector.dot(vector_neighbors[i]);

                Eigen::Vector3d point_end = frame->p_state->rotation.inverse() * point_world -
                                            frame->p_state->rotation.inverse() * frame->p_state->translation;
                // 使用解析解的方式构造factors
            }
        }
        if (num_residuals >= options_.max_num_residuals) {
            break;
        }
    }
}

// 类似于kdtree的寻找最近的5个点，拟合平面
std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> LidarOdom::searchNeighbors(
    VoxelHashMap& map, const Eigen::Vector3d& point_body, int nb_voxel_visited, int size_voxel,
    int max_num_neighborhoods, int threshold_voxel_capacity, std::vector<Voxel>* voxels) {
    // 预分配空间
    if (voxels != nullptr) {
        voxels->reserve(max_num_neighborhoods);
    }
    // 体素化
    short grid_x = static_cast<short>(point_body[0] / size_voxel);
    short grid_y = static_cast<short>(point_body[1] / size_voxel);
    short grid_z = static_cast<short>(point_body[2] / size_voxel);
    // 小堆队列，距离最近的放在queue的头部
    priority_queue_t priority_queue;

    Voxel voxel_tmp(grid_x, grid_y, grid_z);
    // 遍历周围的体素
    for (short kxx = grid_x - nb_voxel_visited; kxx < grid_x + nb_voxel_visited + 1; ++kxx) {
        for (short kyy = grid_y - nb_voxel_visited; kyy < grid_y + nb_voxel_visited + 1; ++kyy) {
            for (short kzz = grid_z - nb_voxel_visited; kzz < grid_z + nb_voxel_visited + 1; ++kzz) {
                voxel_tmp.x = kxx;
                voxel_tmp.y = kyy;
                voxel_tmp.z = kzz;
                auto iter = voxel_map.find(voxel_tmp);
                // 找到相邻的体素
                if (iter != voxel_map.end()) {
                    const auto& voxel_block = iter.value();
                    if (voxel_block.GetNumPoints() < threshold_voxel_capacity) {
                        continue;
                    }
                    // 计算点到临近体素中点的距离
                    for (int i = 0; i < voxel_block.GetNumPoints(); ++i) {
                        auto neighbor_point = voxel_block.points[i];
                        double distance = (neighbor_point - point_body).norm();
                        if (priority_queue.size() == max_num_neighborhoods) {
                            if (std::get<0>(priority_queue.top()) > distance) {
                                priority_queue.pop();
                                priority_queue.emplace(distance, neighbor_point, voxel_tmp);
                            }
                        } else {
                            priority_queue.emplace(distance, neighbor_point, voxel_tmp);
                        }
                    }
                }
            }
        }
    }
    auto queue_size = priority_queue.size();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> closest_neighbors(queue_size);
    if (voxels != nullptr) {
        voxels->resize(queue_size);
    }
    for (auto i = 0; i < queue_size; ++i) {
        // 0 distance 1 point 2 voxel
        closest_neighbors[queue_size - i - 1] = std::get<1>(priority_queue.top());
        if (voxels != nullptr) {
            (*voxels)[queue_size - i - 1] = std::get<2>(priority_queue.top());
        }
        priority_queue.pop();
    }
    return closest_neighbors;
}

// 把临近的点都考虑进来，计算点的均值，利用PCA的主成分分析
Neighborhood LidarOdom::computeNeightborhoodsDistribution(
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& neighborhoods) {
    Neighborhood neighborhood;
    // 重心
    Eigen::Vector3d barycenter(0, 0, 0);
    for (const auto& point : neighborhoods) {
        barycenter += point;
    }
    barycenter /= (double)neighborhoods.size();
    neighborhood.center = barycenter;
    // 协方差
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    // 协方差的定义，以及计算了协方差的上三角
    for (auto& point : neighborhoods) {
        for (int k = 0; k < 3; k++) {
            for (int l = k; l < 3; ++l) {
                covariance(k, l) += (point(k) - barycenter(k)) * (point(l) - barycenter(l));
            }
        }
    }
    covariance(1, 0) = covariance(0, 1);
    covariance(2, 0) = covariance(0, 2);
    covariance(2, 1) = covariance(1, 2);

    neighborhood.covariance = covariance;
    // 计算协方差的特征向量
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance);
    // 每个特征值都有对应的特征向量，特征值的顺序也是从小到大的排列
    // 协方差矩阵的特征向量对应的特征值越大，表示在该方向上的数据变化越大。
    Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());
    neighborhood.normal = normal;
    double sigmal_1 = std::sqrt(std::abs(es.eigenvalues()[2]));
    double sigmal_2 = std::sqrt(std::abs(es.eigenvalues()[1]));
    double sigmal_3 = std::sqrt(std::abs(es.eigenvalues()[0]));
    neighborhood.a2D = (sigmal_2 - sigmal_3) / sigmal_1;

    return neighborhood;
}
}  // namespace ctlio