// TODO: license

#include "wut_velma_effort_controller/wut_velma_model.hpp"
#include "wut_velma_effort_controller/wut_velma_types.hpp"
#include <Eigen/src/Geometry/Transform.h>
#include <cstddef>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <sstream>
#include <algorithm>

#include <rclcpp/parameter_client.hpp>

#include <cassert>

#include <Eigen/Dense>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

using Matrix6x  = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using Matrix12x  = Eigen::Matrix<double, 12, Eigen::Dynamic>;

class DualTcpJacobiansNoAlloc
{
public:
    DualTcpJacobiansNoAlloc(
        const pinocchio::Model & model,
        const std::array<pinocchio::FrameIndex, EEcount>& tcp_frame_ids)
    : nv_(model.nv),
        tcp_frame_ids_(tcp_frame_ids),
        J1_(6, model.nv),
        J2_(6, model.nv),
        J12_(12, model.nv)
    {
        J1_.setZero();
        J2_.setZero();
        J12_.setZero();
    }

    // Jacobian is calculated in local frame. In cart imp stiffness and error are calculated
    // in local frame.
    void compute(
        const pinocchio::Model & model,
        pinocchio::Data & data,
        const Eigen::Ref<const Eigen::VectorXd> & q,
        pinocchio::ReferenceFrame rf = pinocchio::LOCAL)
    {
        assert(q.size() == model.nq);
        assert(model.nv == nv_);

        pinocchio::computeJointJacobians(model, data, q);

        pinocchio::updateFramePlacements(model, data);

        // Pinocchio wymaga wyzerowanego bufora J.
        J1_.setZero();
        J2_.setZero();

        pinocchio::getFrameJacobian(
        model,
        data,
        tcp_frame_ids_[0],
        rf,
        J1_);

        pinocchio::getFrameJacobian(
        model,
        data,
        tcp_frame_ids_[1],
        rf,
        J2_);

        // Stack 12 x nv: [J_tcp1; J_tcp2]
        J12_.topRows<6>().noalias() = J1_;
        J12_.bottomRows<6>().noalias() = J2_;
    }

    const Matrix12x & J12() const
    {
        return J12_;
    }

private:
    int nv_;

    std::array<pinocchio::FrameIndex, EEcount> tcp_frame_ids_;

    Matrix6x J1_;
    Matrix6x J2_;
    Matrix12x J12_;
};

class TwoTcpPoseNoAlloc
{
public:
    TwoTcpPoseNoAlloc(const std::array<pinocchio::FrameIndex, EEcount>& tcp_frame_ids)
    : tcp_frame_ids_(tcp_frame_ids)
    {
        p1_.setZero();
        p2_.setZero();
        q1_.setIdentity();
        q2_.setIdentity();
    }

    void compute(
        const pinocchio::Model & model,
        pinocchio::Data & data,
        const Eigen::Ref<const Eigen::VectorXd> & q)
    {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);

        const pinocchio::SE3 & T1 = data.oMf[tcp_frame_ids_[0]];
        const pinocchio::SE3 & T2 = data.oMf[tcp_frame_ids_[1]];

        p1_.noalias() = T1.translation();
        p2_.noalias() = T2.translation();

        q1_ = Eigen::Quaterniond(T1.rotation());
        q2_ = Eigen::Quaterniond(T2.rotation());

        q1_.normalize();
        q2_.normalize();
    }

    const Eigen::Vector3d & p1() const { return p1_; }
    const Eigen::Vector3d & p2() const { return p2_; }

    const Eigen::Quaterniond & q1() const { return q1_; }
    const Eigen::Quaterniond & q2() const { return q2_; }

    const pinocchio::SE3 & T1(const pinocchio::Data & data) const
    {
        return data.oMf[tcp_frame_ids_[0]];
    }

    const pinocchio::SE3 & T2(const pinocchio::Data & data) const
    {
        return data.oMf[tcp_frame_ids_[1]];
    }

private:
    std::array<pinocchio::FrameIndex, EEcount> tcp_frame_ids_;

    Eigen::Vector3d p1_;
    Eigen::Vector3d p2_;
    Eigen::Quaterniond q1_;
    Eigen::Quaterniond q2_;
};

WutVelmaModel::WutVelmaModel(const std::string& urdf_xml,
        const std::vector<std::string> & joint_names,
        const std::vector<std::string> & link_names)
: fk_links_(link_names)
{
    // Initialize pinocchio
    pinocchio::Model model_full;
    pinocchio::urdf::buildModelFromXML(urdf_xml, model_full);

    // Build reduced model;
    std::vector<pinocchio::JointIndex> list_of_joints_to_lock;
    Eigen::VectorXd reference_configuration(model_full.nq);
    reference_configuration.setZero();

    for (pinocchio::JointIndex jid = 1; jid < model_full.joints.size(); ++jid)
    {
        const auto & joint_name = model_full.names[jid];
        std::cout
            << "joint[" << jid << "] "
            << joint_name
            << " nq=" << model_full.joints[jid].nq()
            << " nv=" << model_full.joints[jid].nv()
            << " idx_q=" << model_full.joints[jid].idx_q()
            << " idx_v=" << model_full.joints[jid].idx_v()
            << std::endl;
        if (std::find(joint_names.begin(), joint_names.end(), joint_name) == joint_names.end()) {
            // This joint can be reduced
            list_of_joints_to_lock.push_back(jid);
        }
    }

    p_model_ = pinocchio::buildReducedModel(model_full,
                                            list_of_joints_to_lock, reference_configuration);

    p_data_.emplace(p_model_);
    p_q_.emplace(pinocchio::neutral(p_model_));
    p_q_->setZero();

    // Create a map pinocchio -> "V", where "V" is a list of 15 main joints of WUT Velma
    for (std::size_t iV = 0; iV < joint_names.size(); ++iV) {
        auto jid = p_model_.getJointId(std::string(joint_names[iV]));

        if (jid > 0) {
            auto idx_q = p_model_.joints[jid].idx_q();
            p2v_map_[iV].first = idx_q;
            p2v_map_[iV].second = iV;
            std::cout << "WutVelmaModel: mapping joint \"" << joint_names[iV] << "\": " << iV <<
                        " -> jid(" << jid << "), idx_q(" << idx_q << ")" << std::endl;
        }
        else {
            std::stringstream ss;
            ss << "ERROR: could not find joint " << joint_names[iV] << " in pinocchio / URDF model.";
            std::cout << ss.str() << std::endl;
            throw std::runtime_error(ss.str());
        }
    }

    std::array<pinocchio::FrameIndex, EEcount> tcp_ids;
    tcp_ids[EEidxL] = p_model_.getFrameId("left_arm_7_link");
    tcp_ids[EEidxR] = p_model_.getFrameId("right_arm_7_link");

    std::cout << "left_tcp_id: " << tcp_ids[EEidxL] << std::endl;
    std::cout << "right_tcp_id: " << tcp_ids[EEidxR] << std::endl;

    dual_tcp_jac_ = std::make_shared<DualTcpJacobiansNoAlloc>(p_model_, tcp_ids);
    two_tcp_pose_ = std::make_shared<TwoTcpPoseNoAlloc>(tcp_ids);

    // Setup FK
    for (const auto & link_name : fk_links_) {
        fk_links_id_.push_back( p_model_.getFrameId(link_name) );
        fk_.push_back( Eigen::Affine3d::Identity() );
    }

    // TODO: wyciąć z macierzy masy dwie macierze - dla każdego ramienia
    // TODO: uwzględnić zmienne narzędzie
}

void WutVelmaModel::setJointPosition(const VVector& joint_position) {
    // Remap joints from "V" to pinocchio
    for (auto & p2v : p2v_map_) {
        (*p_q_)[p2v.first] = joint_position[p2v.second];
    }
}

// TODO: add outputs:
// - forward kinematics for all links
// - jacobian
void WutVelmaModel::calculateMassMatrix(VMatrix& mass_matrix)
{

    // CRBA -> M (upper triangle)
    pinocchio::crba(p_model_, *p_data_, *p_q_);

    // Make it symmetric
    p_data_->M.triangularView<Eigen::StrictlyLower>() =
        p_data_->M.transpose().triangularView<Eigen::StrictlyLower>();

    // Remap from pinocchio to "V"
    auto M = p_data_->M;
    for (auto & p2v_1 : p2v_map_) {
        for (auto & p2v_2 : p2v_map_) {
            mass_matrix(p2v_1.second, p2v_2.second) = M(p2v_1.first, p2v_2.first);
        }
    }
}

void WutVelmaModel::calculateJacobian(Jacobian& jacobian)
{
    dual_tcp_jac_->compute(p_model_, *p_data_, *p_q_);
    auto J12 = dual_tcp_jac_->J12();

    for (auto & p2v_1 : p2v_map_) {
        for (size_t i = 0; i < 12; ++i) {
            jacobian(i, p2v_1.second) = J12(i, p2v_1.first);
        }
    }
}

void WutVelmaModel::calculateTcpFk(Eigen::Affine3d& p1, Eigen::Affine3d& p2)
{
    two_tcp_pose_->compute(p_model_, *p_data_, *p_q_);
    auto T1 = two_tcp_pose_->T1(*p_data_);
    auto T2 = two_tcp_pose_->T2(*p_data_);

    p1.linear().noalias() = T1.rotation();
    p1.translation().noalias() = T1.translation();

    p2.linear().noalias() = T2.rotation();
    p2.translation().noalias() = T2.translation();
}

void WutVelmaModel::calculateFk()
{
    pinocchio::forwardKinematics(p_model_, *p_data_, *p_q_);
    pinocchio::updateFramePlacements(p_model_, *p_data_);
    for (size_t i = 0; i < fk_links_id_.size(); ++i) {
        const pinocchio::SE3 & T = p_data_->oMf[fk_links_id_[i]];
        fk_[i].linear().noalias() = T.rotation();
        fk_[i].translation().noalias() = T.translation();
    }
}

const std::vector<Eigen::Affine3d>& WutVelmaModel::getFk() const {
    return fk_;
}
