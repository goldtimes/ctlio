#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "lio/lio_utils.hh"

namespace ctlio {
class PoseParameterization : public ceres::LocalParameterization {
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const {
        return 7;
    }
    virtual int LocalSize() const {
        return 6;
    }
};

class RotationParameterization : public ceres::LocalParameterization {
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const {
        return 4;
    }
    virtual int LocalSize() const {
        return 3;
    }
};
}  // namespace ctlio