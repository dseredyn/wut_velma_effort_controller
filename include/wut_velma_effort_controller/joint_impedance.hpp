// TODO: license

#ifndef JOINT_IMPEDANCE_HPP_
#define JOINT_IMPEDANCE_HPP_

#include <Eigen/Dense>
#include "wut_velma_effort_controller/wut_velma_types.hpp"

class JointImpedance {
  public:
    JointImpedance();

    [[nodiscard]] bool calculate(const VVector& joint_position,
        const VVector& joint_position_cmd,
        const VVector& stiffness,
        const VVector& joint_velocity,
        const VVector& nullspace_torque_cmd,
        const VMatrix& mass_matrix,
        VVector& out_joint_torque_command) const;

  protected:
    float damping_;
};

#endif  // JOINT_IMPEDANCE_HPP_
