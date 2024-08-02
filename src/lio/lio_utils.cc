#include "lio/lio_utils.hh"

namespace ctlio {
State::State(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_, const Eigen::Vector3d &velocity_,
             const Eigen::Vector3d &ba_, const Eigen::Vector3d &bg_)
    : rotation{rotation_}, translation{translation_}, velocity{velocity_}, ba{ba_}, bg{bg_} {
}

State::State(const std::shared_ptr<State> state_temp, bool copy) {
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

CloudFrame::CloudFrame(std::vector<point3D> &points_world_tmp, std::vector<point3D> &points_lidar_tmp,
                       std::shared_ptr<State> p_state_) {
    points_world.insert(points_world.end(), points_world_tmp.begin(), points_world_tmp.end());
    points_lidar.insert(points_lidar.end(), points_lidar_tmp.begin(), points_lidar_tmp.end());

    p_state.reset(new State(p_state_, true));
    success = true;
}

CloudFrame::CloudFrame(std::shared_ptr<CloudFrame> p_cloud_frame) {
    time_frame_begin = p_cloud_frame->time_frame_begin;
    time_frame_end = p_cloud_frame->time_frame_end;
    frame_id = p_cloud_frame->frame_id;
    p_state.reset(new State(p_cloud_frame->p_state, true));

    points_world.insert(points_world.end(), p_cloud_frame->points_world.begin(), p_cloud_frame->points_world.end());
    points_lidar.insert(points_lidar.end(), p_cloud_frame->points_lidar.begin(), p_cloud_frame->points_lidar.end());
    dt_offset = p_cloud_frame->dt_offset;
    success = p_cloud_frame->success;
}

void CloudFrame::release() {
    std::vector<point3D>().swap(points_world);
    std::vector<point3D>().swap(points_lidar);
    p_state.reset();
}

/**
 * @brief 根据不同的运动去畸变模式进行处理
 * @param rotation_begin 世界坐标系下的位姿
 */

void transformPoint(MotionCompensation motion_compensation, point3D &point, const Eigen::Quaterniond &rotation_begin,
                    const Eigen::Vector3d &trans_begin, const Eigen::Quaterniond &rotation_end,
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
    // 转换到body坐标系
    point.point_world = R * (R_IL * point.point + t_IL) + t;
}
}  // namespace ctlio