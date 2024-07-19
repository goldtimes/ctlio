#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>

namespace ctlio {
class Odom {
   public:
    Odom() = default;
    Odom(double time, double left_pulse, double right_pulse)
        : timestamp_(time), left_pulse_(left_pulse), right_pulse_(right_pulse) {
    }
    double timestamp_ = 0.0;
    double left_pulse_ = 0.0;
    double right_pulse_ = 0.0;
};
}  // namespace ctlio