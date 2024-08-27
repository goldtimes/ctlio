#include "lio/lio_utils.hh"

namespace ctlio {
state::state(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_, const Eigen::Vector3d &velocity_,
             const Eigen::Vector3d &ba_, const Eigen::Vector3d &bg_)
    : rotation{rotation_}, translation{translation_}, velocity{velocity_}, ba{ba_}, bg{bg_} {
}

state::state(const state *state_temp, bool copy) {
    if (copy) {
        rotation = state_temp->rotation;
        translation = state_temp->translation;

        rotation_begin = state_temp->rotation_begin;
        translation_begin = state_temp->translation_begin;

        velocity = state_temp->velocity;
        ba = state_temp->ba;
        bg = state_temp->bg;

        velocity_begin = state_temp->velocity_begin;
        ba_begin = state_temp->ba_begin;
        bg_begin = state_temp->bg_begin;
    } else {
        rotation = state_temp->rotation;
        translation = state_temp->translation;

        rotation_begin = state_temp->rotation;
        translation_begin = state_temp->translation;

        velocity_begin = state_temp->velocity;
        ba_begin = state_temp->ba;
        bg_begin = state_temp->bg;

        velocity = state_temp->velocity;
        ba = state_temp->ba;
        bg = state_temp->bg;
    }
}

void state::release() {
}

//   --------------------------------------     //
cloudFrame::cloudFrame(std::vector<point3D> &point_surf_, std::vector<point3D> &const_surf_, state *p_state_) {
    point_surf.insert(point_surf.end(), point_surf_.begin(), point_surf_.end());
    const_surf.insert(const_surf.end(), const_surf_.begin(), const_surf_.end());

    // p_state = p_state_;
    p_state = new state(p_state_, true);

    success = true;
}

cloudFrame::cloudFrame(cloudFrame *p_cloud_frame) {
    time_frame_begin = p_cloud_frame->time_frame_begin;
    time_frame_end = p_cloud_frame->time_frame_end;

    frame_id = p_cloud_frame->frame_id;

    // p_state = p_cloud_frame->p_state;
    p_state = new state(p_cloud_frame->p_state, true);

    point_surf.insert(point_surf.end(), p_cloud_frame->point_surf.begin(), p_cloud_frame->point_surf.end());
    const_surf.insert(const_surf.end(), p_cloud_frame->const_surf.begin(), p_cloud_frame->const_surf.end());

    dt_offset = p_cloud_frame->dt_offset;

    success = p_cloud_frame->success;
}

void cloudFrame::release() {
    std::vector<point3D>().swap(point_surf);
    std::vector<point3D>().swap(const_surf);

    p_state = nullptr;
}
/**
 * @brief 根据不同的运动去畸变模式进行处理
 * @param rotation_begin 世界坐标系下的位姿
 */

void transformPoint(MotionCompensation motion_compensation, point3D &point, const Eigen::Quaterniond &rotation_begin,
                    const Eigen::Quaterniond &rotation_end, const Eigen::Vector3d &trans_begin,
                    const Eigen::Vector3d &trans_end, const Eigen::Matrix3d &R_IL, const Eigen::Vector3d &t_IL) {
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    double alpha_time = point.alpha_time;
    switch (motion_compensation) {
        case MotionCompensation::NONE:
        // 匀速运动
        case MotionCompensation::CONSTANT_VELOCITY:
            R = rotation_end.toRotationMatrix();
            t = trans_end;
            break;
        case MotionCompensation::CONTINUOUS:
        // 插值
        case MotionCompensation::ITERATIVE:
            R = rotation_begin.slerp(alpha_time, rotation_end).normalized().toRotationMatrix();
            t = trans_begin * (1 - alpha_time) + trans_end * alpha_time;
            break;
    }
    // 转换到odom坐标系
    point.point = R * (R_IL * point.raw_point + t_IL) + t;
}

void GridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling) {
    keypoints.resize(0);
    std::vector<point3D> frame_sub;
    frame_sub.resize(frame.size());
    for (int i = 0; i < (int)frame_sub.size(); i++) {
        frame_sub[i] = frame[i];
    }
    SubSampleFrame(frame_sub, size_voxel_subsampling);
    keypoints.reserve(frame_sub.size());
    for (int i = 0; i < (int)frame_sub.size(); i++) {
        keypoints.push_back(frame_sub[i]);
    }
}

void SubSampleFrame(std::vector<point3D> &frame, double size_voxel) {
    std::tr1::unordered_map<voxel, std::vector<point3D>, std::hash<voxel>> grid;
    for (int i = 0; i < (int)frame.size(); i++) {
        auto kx = static_cast<short>(frame[i].point[0] / size_voxel);
        auto ky = static_cast<short>(frame[i].point[1] / size_voxel);
        auto kz = static_cast<short>(frame[i].point[2] / size_voxel);
        grid[voxel(kx, ky, kz)].push_back(frame[i]);
    }
    frame.resize(0);
    int step = 0;
    for (const auto &n : grid) {
        if (n.second.size() > 0) {
            frame.push_back(n.second[0]);
            step++;
        }
    }
}

double AngularDistance(const Eigen::Vector3d &qa, const Eigen::Vector3d &qb) {
    Eigen::Quaterniond q_a = Eigen::Quaterniond(Sophus::SO3d::exp(qa).matrix());
    Eigen::Quaterniond q_b = Eigen::Quaterniond(Sophus::SO3d::exp(qb).matrix());
    q_a.normalize();
    q_b.normalize();
    Eigen::Matrix3d rot_a = q_a.toRotationMatrix();
    Eigen::Matrix3d rot_b = q_b.toRotationMatrix();
    double norm = ((rot_a * rot_b.transpose()).trace() - 1) / 2;
    norm = std::acos(norm) * 180 / M_PI;
    return norm;
}
double AngularDistance(const Eigen::Quaterniond &qa, const Eigen::Quaterniond &qb) {
    Eigen::Matrix3d rot_a = qa.toRotationMatrix();
    Eigen::Matrix3d rot_b = qb.toRotationMatrix();
    double norm = ((rot_a * rot_b.transpose()).trace() - 1) / 2;
    norm = std::acos(norm) * 180 / M_PI;
    return norm;
}
}  // namespace ctlio