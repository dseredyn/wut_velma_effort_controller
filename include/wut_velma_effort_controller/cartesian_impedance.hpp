# pragma once

#include <Eigen/Dense>
#include "wut_velma_effort_controller/wut_velma_types.hpp"

class CartesianImpedance {
public:
    CartesianImpedance();

    [[nodiscard]] bool calculate(
        const VVector& joint_velocity,
        const VMatrix& mass_matrix,
        const Jacobian& jacobian,
        const std::array<Eigen::Affine3d, EEcount>& cart_position,
        const std::array<Eigen::Affine3d, EEcount>& cart_position_cmd,
        const Vector2E& Kc,
        const Vector2E& Dxi,
        const VVector& nullspace_torque_cmd,
        VVector& out_joint_torque_command) const;

protected:
};
