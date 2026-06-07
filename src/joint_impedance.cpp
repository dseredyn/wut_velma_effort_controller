// TODO: license

#include "wut_velma_effort_controller/joint_impedance.hpp"

JointImpedance::JointImpedance()
    :damping_(0.7)
    // m_velma_()
{}

bool JointImpedance::calculate(const VVector& joint_position,
      const VVector& joint_position_cmd,
      const VVector& stiffness,
      const VVector& joint_velocity,
      const VVector& nullspace_torque_cmd,
      const VMatrix& mass_matrix,
      VVector& out_joint_torque_command) const {

    // This can be 0: nullspace_torque_cmd

    VVector joint_error_;
    VMatrix tmpNN_;
    VVector k0_;
    VMatrix d_, q_;
    Eigen::GeneralizedSelfAdjointEigenSolver<VMatrix > es_;

    joint_error_.noalias() = joint_position_cmd - joint_position;
    out_joint_torque_command.noalias() = stiffness.cwiseProduct(joint_error_);


    // TODO: adjust torso stiffness
    // Add some stiffness for torso, if the arms are widely spread
    // double stiffness_torso_add = 0.0;
    // if (add_stiffness_torso_ == 1) {
    //   geometry_msgs::Pose r_wrist_pose;
    //   geometry_msgs::Pose l_wrist_pose;
    //   const double wrist_dist_min = 0.25;
    //   const double wrist_dist_max = 1.25;
    //   const double max_stiffness_torso = stiffness(0)*0.5;
    //   if (port_r_wrist_pose_.read(r_wrist_pose) == RTT::NewData) {
    //     double wrist_dist = sqrt(r_wrist_pose.position.x*r_wrist_pose.position.x +
    //                               r_wrist_pose.position.y*r_wrist_pose.position.y);
    //     stiffness_torso_add += max_stiffness_torso * std::min(1.0,
    //                               std::max(0.0,
    //                               wrist_dist - wrist_dist_min) / (wrist_dist_max - wrist_dist_min));
    //   }
    //   if (port_l_wrist_pose_.read(l_wrist_pose) == RTT::NewData) {
    //     double wrist_dist = sqrt(l_wrist_pose.position.x*l_wrist_pose.position.x +
    //                               l_wrist_pose.position.y*l_wrist_pose.position.y);
    //     stiffness_torso_add += max_stiffness_torso * std::min(1.0,
    //                               std::max(0.0,
    //                               wrist_dist - wrist_dist_min) / (wrist_dist_max - wrist_dist_min));
    //   }
    // }

    // TODO: adjust torso stiffness
    // double original_torso_stiffness = stiffness(0);
    // stiffness(0) = stiffness(0) + stiffness_torso_add;

    // debug print
    // if (fabs(stiffness(0) - prev_torso_stiffness_) > 100) {
    //   prev_torso_stiffness_ = stiffness(0);
    //   std::cout << "JointImpedance: new stiffness for torso: " << stiffness(0) << std::endl;
    // }

    tmpNN_ = stiffness.asDiagonal();

    // TODO: adjust torso stiffness
    // stiffness(0) = original_torso_stiffness;

    es_.compute(tmpNN_, mass_matrix);

    q_ = es_.eigenvectors().inverse();

    k0_ = es_.eigenvalues();

    tmpNN_ = k0_.cwiseAbs().cwiseSqrt().asDiagonal();

    // d_.noalias() = 2.0 * q_.transpose() * 0.7 * tmpNN_ * q_;    // hard-coded damping is replaced by ROS param damping
    d_.noalias() = 2.0 * q_.transpose() * damping_ * tmpNN_ * q_;

    if (!out_joint_torque_command.allFinite()) {
        // TODO
        return false;
        // m_fabric_logger << "Non finite output form stiffness" << FabricLogger::End();
        // error();
    }

    out_joint_torque_command.noalias() -= d_ * joint_velocity;

    if (!out_joint_torque_command.allFinite()) {
        // TODO
        return false;
        // m_fabric_logger << "Non finite output form damping" << FabricLogger::End();
        // error();
    }

    out_joint_torque_command.noalias() += nullspace_torque_cmd;

    if (!out_joint_torque_command.allFinite()) {
        // TODO
        return false;
        // m_fabric_logger << "Non finite output form nullspace" << FabricLogger::End();
        // error();
    }
    return true;
}
