// TODO: license

#include "wut_velma_effort_controller/cartesian_impedance.hpp"

#include <cstddef>
#include <array>
#include <Eigen/Dense>
#include "wut_velma_effort_controller/wut_velma_types.hpp"

#include <iostream>

CartesianImpedance::CartesianImpedance()
{}

// TODO: this function should be const
bool CartesianImpedance::calculate(
        // const VVector& joint_position,
        const VVector& joint_velocity,
        const VMatrix& mass_matrix,
        const Jacobian& jacobian,
        const std::array<Eigen::Affine3d, EEcount>& cart_position,
        const std::array<Eigen::Affine3d, EEcount>& cart_position_cmd,
        // const std::array<Eigen::Affine3d, EEcount>& cart_tool_cmd,
        const Vector2E& Kc,
        const Vector2E& Dxi,
        const VVector& nullspace_torque_cmd,
        VVector& out_joint_torque_command) const
{

    // This can be 0: nullspace_torque_cmd

    Eigen::PartialPivLU<VMatrix> lu_;
    Eigen::PartialPivLU<Matrix2E> luKK_;
    Eigen::GeneralizedSelfAdjointEigenSolver<Matrix2E> es_;

    // r_cmd --> cart_position_cmd

    // for (size_t i = 0; i < EEcount; i++) {
    //   if (port_tool_position_command_[i]->read(pos) == RTT::NewData) {
    //     tools[i](0) = pos.position.x;
    //     tools[i](1) = pos.position.y;
    //     tools[i](2) = pos.position.z;

    //     tools[i](3) = pos.orientation.w;
    //     tools[i](4) = pos.orientation.x;
    //     tools[i](5) = pos.orientation.y;
    //     tools[i](6) = pos.orientation.z;

    //     if (enable_rt_logging_) {
    //       m_fabric_logger << "tool " << i << " " << pos.position.x << " " << pos.position.y
    //                       << " " << pos.position.z << " " << pos.orientation.x
    //                       << " " << pos.orientation.y << " " << pos.orientation.z
    //                       << " " << pos.orientation.w << FabricLogger::End();
    //       //rt_logging_output = true;
    //     }
    //   }

    //   cartesian_trajectory_msgs::CartesianImpedance impedance;
    //   if (port_cartesian_impedance_command_[i]->read(impedance)
    //       == RTT::NewData) {
    //     Kc(i * 6 + 0) = impedance.stiffness.force.x;
    //     Kc(i * 6 + 1) = impedance.stiffness.force.y;
    //     Kc(i * 6 + 2) = impedance.stiffness.force.z;
    //     Kc(i * 6 + 3) = impedance.stiffness.torque.x;
    //     Kc(i * 6 + 4) = impedance.stiffness.torque.y;
    //     Kc(i * 6 + 5) = impedance.stiffness.torque.z;

    //     Dxi(i * 6 + 0) = impedance.damping.force.x;
    //     Dxi(i * 6 + 1) = impedance.damping.force.y;
    //     Dxi(i * 6 + 2) = impedance.damping.force.z;
    //     Dxi(i * 6 + 3) = impedance.damping.torque.x;
    //     Dxi(i * 6 + 4) = impedance.damping.torque.y;
    //     Dxi(i * 6 + 5) = impedance.damping.torque.z;
    //   }
    // }

    bool verbose = false;
    // M --> mass_matrix
    if (!mass_matrix.allFinite()) {
        std::cout << "M contains NaN or inf" << std::endl;
        return false;
    }

    // calculate robot data
    // robot_->jacobian(J, joint_position, &tools[0]);
    // J --> jacobian
    if (!jacobian.allFinite()) {
        std::cout << "J contains NaN or inf" << std::endl;
        return false;
    }

    // robot_->fkin(r, joint_position, &tools[0]);
    // r --> cart_position

    // if (enable_rt_logging_) {
    //   for (int i = 0; i < EEcount; i++) {
    //     Eigen::Quaterniond q( r[i].rotation() );
    //     Eigen::Vector3d t(r[i].translation());
    //     m_fabric_logger << "pos " << i << " " << t(0) << " " << t(1)
    //                     << " " << t(2) << " " << q.x()
    //                     << " " << q.y() << " " << q.z()
    //                     << " " << q.w() << FabricLogger::End();
    //   }
    //   //rt_logging_output = true;
    // }

    JacobianT jacobianT = jacobian.transpose();
    // JT --> jacobianT

    lu_.compute(mass_matrix);
    VMatrix Mi = lu_.inverse();
    // VMatrix P;

    if (!Mi.allFinite()) {
        std::cout << "Mi contains NaN or inf" << std::endl;
        return false;
    }

    Vector2E p;

    // calculate stiffness component
    for (size_t i = 0; i < EEcount; i++) {
      Eigen::Affine3d tmp;
      tmp = cart_position[i].inverse() * cart_position_cmd[i];

      p(i * 6) = tmp.translation().x();
      p(i * 6 + 1) = tmp.translation().y();
      p(i * 6 + 2) = tmp.translation().z();

      Eigen::Quaternion<double> quat(tmp.rotation());
      p(i * 6 + 3) = quat.x();
      p(i * 6 + 4) = quat.y();
      p(i * 6 + 5) = quat.z();
    }

    Vector2E F;

    F.noalias() = (Kc.array() * p.array()).matrix();
    if (verbose) std::cout << "F " << F << std::endl;
    out_joint_torque_command.noalias() = jacobianT * F;

    if (!out_joint_torque_command.allFinite()) {
        std::cout << "joint_torque_command_ contains NaN or inf" << std::endl;
        std::cout << "out_joint_torque_command: " << out_joint_torque_command << std::endl;
        std::cout << "cart_position_cmd[0]: " << cart_position_cmd[0].matrix() << std::endl;
        std::cout << "cart_position[0]: " << cart_position[0].matrix() << std::endl;
        
        std::cout << "Kc: " << Kc << std::endl;
        std::cout << "p: " << p << std::endl;
        std::cout << "F: " << F << std::endl;
        std::cout << "jacobianT: " << jacobianT << std::endl;
        return false;
    }

    // Calculate damping component

    Jacobian tmpKN_;
    tmpKN_.noalias() = jacobian * Mi;
    if (verbose) std::cout << "tmpKN_ " << tmpKN_ << std::endl;
    Matrix2E A;
    A.noalias() = tmpKN_ * jacobianT;
    if (verbose) std::cout << "A " << A << std::endl;
    luKK_.compute(A);
    A = luKK_.inverse();
    if (verbose) std::cout << "A " << A << std::endl;

    Matrix2E tmpKK_ = Kc.asDiagonal();
    if (verbose) std::cout << "tmpKK_ " << tmpKK_ << std::endl;

    // UNRESTRICT_ALLOC;
    es_.compute(tmpKK_, A);
    // RESTRICT_ALLOC;
    Vector2E K0 = es_.eigenvalues();
    luKK_.compute(es_.eigenvectors());
    Matrix2E Q = luKK_.inverse();
    if (verbose) std::cout << "Q " << Q << std::endl;

    tmpKK_ = Dxi.asDiagonal();
    if (verbose) std::cout << "tmpKK_ " << tmpKK_ << std::endl;

    Matrix2E Dc;
    Dc.noalias() = Q.transpose() * tmpKK_;
    tmpKK_ = K0.cwiseSqrt().asDiagonal();
    if (verbose) std::cout << "tmpKK_ " << tmpKK_ << std::endl;
    Matrix2E tmpKK2_;
    tmpKK2_.noalias() = Dc *  tmpKK_;
    if (verbose) std::cout << "tmpKK2_ " << tmpKK2_ << std::endl;
    Dc.noalias() = tmpKK2_ * Q;
    if (verbose) std::cout << "Dc " << Dc << std::endl;
    Vector2E tmpK_;
    tmpK_.noalias() = jacobian * joint_velocity;
    if (verbose) std::cout << "tmpK_ " << tmpK_ << std::endl;
    F.noalias() = Dc * tmpK_;

    if (!F.allFinite()) {
        std::cout << "F contains NaN or inf: " << F << std::endl;
        return false;
    }

    out_joint_torque_command.noalias() -= jacobianT * F;



    // Calculate null-space component
    tmpKN_.noalias() = jacobian * Mi;
    tmpKK_.noalias() = tmpKN_ * jacobianT;
    luKK_.compute(tmpKK_);
    tmpKK_ = luKK_.inverse();
    JacobianT tmpNK_;
    tmpNK_.noalias() = Mi * jacobianT;
    JacobianT Ji;
    Ji.noalias() = tmpNK_ * tmpKK_;

    VMatrix P;
    P.noalias() = VMatrix::Identity();
    P.noalias() -=  jacobian.transpose() * A * jacobian * Mi;

    if (!P.allFinite()) {
        std::cout << "P contains NaN or inf" << std::endl;
        return false;
    }

    out_joint_torque_command.noalias() += P * nullspace_torque_cmd;

    // write outputs
    // UNRESTRICT_ALLOC;

    // if (enable_rt_logging_) {
    //   m_fabric_logger << "torque ";
    //   for (int i = 0; i < DOFS; i++) {
    //     if (i > 0) {
    //       m_fabric_logger << " ";
    //     }
    //     m_fabric_logger << joint_torque_command_(i);
    //   }
    //   m_fabric_logger << FabricLogger::End();
    // }

    // for (size_t i = 0; i < EEcount; i++) {
    //   geometry_msgs::Pose pos;
    //   tf::poseEigenToMsg(r[i], pos);
    //   port_cartesian_position_[i]->write(pos);
    // }

    return true;






    // VVector joint_error_;
    // VMatrix tmpNN_;
    // VVector k0_;
    // VMatrix d_, q_;
    // Eigen::GeneralizedSelfAdjointEigenSolver<VMatrix > es_;

    // joint_error_.noalias() = joint_position_cmd - joint_position;
    // out_joint_torque_command.noalias() = stiffness.cwiseProduct(joint_error_);


    // tmpNN_ = stiffness.asDiagonal();

    // // TODO: adjust torso stiffness
    // // stiffness(0) = original_torso_stiffness;

    // es_.compute(tmpNN_, mass_matrix);

    // q_ = es_.eigenvectors().inverse();

    // k0_ = es_.eigenvalues();

    // tmpNN_ = k0_.cwiseAbs().cwiseSqrt().asDiagonal();

    // // d_.noalias() = 2.0 * q_.transpose() * 0.7 * tmpNN_ * q_;    // hard-coded damping is replaced by ROS param damping
    // d_.noalias() = 2.0 * q_.transpose() * damping_ * tmpNN_ * q_;

    // if (!out_joint_torque_command.allFinite()) {
    //     // TODO
    //     return false;
    //     // m_fabric_logger << "Non finite output form stiffness" << FabricLogger::End();
    //     // error();
    // }

    // out_joint_torque_command.noalias() -= d_ * joint_velocity;

    // if (!out_joint_torque_command.allFinite()) {
    //     // TODO
    //     return false;
    //     // m_fabric_logger << "Non finite output form damping" << FabricLogger::End();
    //     // error();
    // }

    // out_joint_torque_command.noalias() += nullspace_torque_cmd;

    // if (!out_joint_torque_command.allFinite()) {
    //     // TODO
    //     return false;
    //     // m_fabric_logger << "Non finite output form nullspace" << FabricLogger::End();
    //     // error();
    // }
    // return true;
}
