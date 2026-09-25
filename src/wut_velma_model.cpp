// TODO: license

#include "wut_velma_effort_controller/wut_velma_model.hpp"
#include "wut_velma_effort_controller/wut_velma_types.hpp"
#include <Eigen/src/Core/Matrix.h>
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

#include <stdexcept>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <moveit/collision_detection/collision_matrix.hpp>

using Matrix6x  = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using Matrix12x  = Eigen::Matrix<double, 12, Eigen::Dynamic>;

bool shouldCheckCollision(
    const collision_detection::AllowedCollisionMatrix& acm,
    const std::string& link1,
    const std::string& link2)
{
    collision_detection::AllowedCollision::Type type;

    if (!acm.getAllowedCollision(link1, link2, type))
    {
        // No ACM entry/default -> collision is not explicitly allowed
        return true;
    }

    return type != collision_detection::AllowedCollision::ALWAYS;
}

CollisionGeom::CollisionGeom(Type type)
: type_(type)
{}

CollisionGeom::Type CollisionGeom::getType() const {
    return type_;
}

CollisionSphere::CollisionSphere()
: CollisionGeom(CollisionGeom::SPHERE)
{}

CollisionSphere::CollisionSphere(double radius, Eigen::Vector3d p)
: CollisionGeom(CollisionGeom::SPHERE)
, radius(radius)
, p(p)
{}

void CollisionSphere::updateReferenceFrame(const Eigen::Affine3d& f) {
    p_g = f * p;
}

CollisionCapsule::CollisionCapsule()
: CollisionGeom(CollisionGeom::CAPSULE)
{}

CollisionCapsule::CollisionCapsule(double radius, Eigen::Vector3d p0, Eigen::Vector3d p1)
: CollisionGeom(CollisionGeom::CAPSULE)
, radius(radius)
, length((p0-p1).norm())
, p0(p0)
, p1(p1)
{}

void CollisionCapsule::updateReferenceFrame(const Eigen::Affine3d& f) {
    p0_g = f * p0;
    p1_g = f * p1;
}

CollisionData::CollisionData()
{}

CollisionData::CollisionData(size_t link0_idx, size_t link1_idx)
: link0_idx(link0_idx)
, link1_idx(link1_idx)
{
}

void CollisionData::swapPoints() {
    const Eigen::Vector3d p_tmp = p0;
    p0 = p1;
    p1 = p_tmp;
}

void CollisionData::calculate(const Eigen::Vector3d& c0, double r0, const Eigen::Vector3d& c1, double r1) {

    Eigen::Vector3d v = c1-c0;
    const double cc_dist = v.norm();
    v = v / cc_dist;
    p0 = c0 + v * r0;
    p1 = c1 - v * r1;
    distance = cc_dist - r0 - r1;
}


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

size_t getIndexOf(const std::string& s, const std::vector<std::string>& v) {
    auto it = std::find(v.begin(), v.end(), s);
    if (it == v.end())
    {
        std::stringstream ss;
        ss << "ERROR: could not find name \"" << s << "\"";
        throw std::runtime_error(ss.str());
    }
    // else
    return std::distance(v.begin(), it);
}

WutVelmaModel::WutVelmaModel(
        const std::string& urdf_xml,
        const std::vector<std::string> & joint_names,
        const std::vector<std::string> & link_names,
        const std::string& srdf_xml,
        double collision_distance,
        std::vector<std::string> collision_link_names,
        std::vector<CollisionGeomSharedPtr> collision_link_geoms,
        const std::vector<std::pair<std::string, std::string>>& collision_pairs)
: fk_links_(link_names),
collision_link_geoms_(collision_link_geoms)
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
        bool is_locked = (std::find(joint_names.begin(), joint_names.end(), joint_name) == joint_names.end());
        std::cout
            << "joint[" << jid << "] " << joint_name << ", "
            << "nq=" << model_full.joints[jid].nq() << ", "
            << "nv=" << model_full.joints[jid].nv() << ", "
            << "idx_q=" << model_full.joints[jid].idx_q() << ", "
            << "idx_v=" << model_full.joints[jid].idx_v() << ", "
            << "locked=" << is_locked
            << std::endl;
        if (is_locked) {
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

    // Parse SRDF, create ACM
    urdf::Model urdf_model;
    if (!urdf_model.initString(urdf_xml))
    {
        std::stringstream ss;
        ss << "ERROR: could not parse URDF.";
        throw std::runtime_error(ss.str());
    }

    srdf::Model srdf_model;
    if (!srdf_model.initString(urdf_model, srdf_xml))
    {
        std::stringstream ss;
        ss << "ERROR: could not parse SRDF.";
        throw std::runtime_error(ss.str());
    }

    std::unique_ptr<collision_detection::AllowedCollisionMatrix> acm_;

    acm_ =
        std::make_unique<collision_detection::AllowedCollisionMatrix>(
            srdf_model);

    // Prepare a list of checks for all self-collision pairs: (c_idx0, l_idx0, c_idx1, l_idx1),
    // where:
    // c_idxX - collision geometry index in collision_link_geoms
    // l_idxX - corresponding link index in link_names

    // std::function<int(const std_msgs::string& link_name)> linkNameToIdx =
    //     [prom, done](const std_msgs::string& link_name)
    //     {
    //     if (!msg || msg->data.empty()) return;

    //     bool expected = false;
    //     if (!done->compare_exchange_strong(expected, true)) return; // already done

    //     prom->set_value(msg->data);
    //     };
    // acm_->removeEntry(const std::string &name1, const std::string &name2)

    for (size_t i = 0; i < collision_link_names.size(); ++i) {
        auto link_name = collision_link_names[i];
        auto index = getIndexOf(link_name, link_names);
        collision_link_indices_.push_back(index);
        std::cout << "link \"" << link_name << "\" index: " << index << std::endl;

        // auto it = std::find(link_names.begin(), link_names.end(), link_name);
        // if (it == link_names.end())
        // {
        //     std::stringstream ss;
        //     ss << "ERROR: could not find link named \"" << link_name << "\"";
        //     throw std::runtime_error(ss.str());
        // } else
        // {
        //     auto index = std::distance(link_names.begin(), it);
        //     collision_link_indices_.push_back(index);
        //     std::cout << "link \"" << link_name << "\" index: " << index << std::endl;
        // }
    }

    // for (size_t i = 0; i < collision_link_names.size(); ++i) {
    //     for (size_t j = 0; j < collision_link_names.size(); ++j) {
    for (size_t i = 0; i < collision_pairs.size(); ++i) {
            // if (collision_link_names[i] == collision_link_names[j]) {
            //     continue;
            // }
            // if (shouldCheckCollision(*acm_, collision_link_names[i], collision_link_names[j])) {
            //     collision_checks_.push_back(
            //         std::array<size_t, 4>{i, collision_link_indices_[i], j, collision_link_indices_[j]} );
            //     // std::cout << "collision_check: " << collision_link_names[i] << ", " << collision_link_names[j] << std::endl;
            //     //collisions_.push_back( CollisionData(collision_link_indices_[i], collision_link_indices_[j]) );
            // }
            auto name0 = collision_pairs[i].first;
            auto index_col0 = getIndexOf(name0, collision_link_names);
            auto index_link0 = collision_link_indices_[index_col0];
            auto name1 = collision_pairs[i].second;
            auto index_col1 = getIndexOf(name1, collision_link_names);
            auto index_link1 = collision_link_indices_[index_col1];
            if (shouldCheckCollision(*acm_, name0, name1)) {
                collision_checks_.push_back(
                    std::array<size_t, 4>{index_col0, index_link0, index_col1, index_link1} );
                // collision_checks_.push_back(
                //     std::array<size_t, 4>{i, collision_link_indices_[i], j, collision_link_indices_[j]} );
                // std::cout << "collision_check: " << collision_link_names[i] << ", " << collision_link_names[j] << std::endl;
                //collisions_.push_back( CollisionData(collision_link_indices_[i], collision_link_indices_[j]) );
            }
        // }
    }
    std::cout << "total collision_checks: " << collision_checks_.size() << std::endl;
}

size_t WutVelmaModel::getCollisionPairsCount() const {
    return collision_checks_.size();
}

double sphereCapsuleDistance( const CollisionSphereSharedPtr& geom0,
                                const CollisionCapsuleSharedPtr& geom1,
                                CollisionData& data)
{
    const Eigen::Vector3d& p = geom0->p_g;
    const Eigen::Vector3d& a = geom1->p0_g;
    const Eigen::Vector3d& b = geom1->p1_g;
    const Eigen::Vector3d ab = b - a;
    const Eigen::Vector3d ap = p - a;
    const double ab2 = ab.squaredNorm();
    // Degenerate segment.
    if (ab2 <= 1e-24) {
        data.calculate(p, geom0->radius, a, geom1->radius);
        return data.distance;
    }
    // TODO: verify
    double t = ap.dot(ab) / ab2;
    t = std::clamp(t, 0.0, 1.0);

    data.calculate(p, geom0->radius, a + t*ab, geom1->radius);
    return data.distance;
}

double capsulesDistance( const CollisionCapsuleSharedPtr& geom0,
                            const CollisionCapsuleSharedPtr& geom1,
                            CollisionData& data)
{
    constexpr double eps = 1e-24;

    const Eigen::Vector3d& p0 = geom0->p0_g;
    const Eigen::Vector3d& p1 = geom0->p1_g;

    const Eigen::Vector3d& q0 = geom1->p0_g;
    const Eigen::Vector3d& q1 = geom1->p1_g;

    const Eigen::Vector3d u = p1 - p0;
    const Eigen::Vector3d v = q1 - q0;
    const Eigen::Vector3d w = p0 - q0;

    const double a = u.dot(u);
    const double b = u.dot(v);
    const double c = v.dot(v);
    const double d = u.dot(w);
    const double e = v.dot(w);

    // Both segments degenerate to points.
    if (a <= eps && c <= eps) {
        data.calculate(p0, geom0->radius, q0, geom1->radius);
        return data.distance;
        //return w.squaredNorm();
    }

    double s;
    double t;

    // First segment is a point.
    if (a <= eps)
    {
        s = 0.0;
        t = std::clamp(e / c, 0.0, 1.0);
    }
    // Second segment is a point.
    else if (c <= eps)
    {
        t = 0.0;
        s = std::clamp(-d / a, 0.0, 1.0);
    }
    else
    {
        const double denom = a * c - b * b;

        // Non-parallel case.
        if (denom > eps * a * c)
            s = std::clamp((b * e - c * d) / denom, 0.0, 1.0);
        else
            s = 0.0;

        // Find t for this s.
        t = (b * s + e) / c;

        // If t lies outside the second segment, clamp it and
        // recompute the optimal s.
        if (t < 0.0)
        {
            t = 0.0;
            s = std::clamp(-d / a, 0.0, 1.0);
        }
        else if (t > 1.0)
        {
            t = 1.0;
            s = std::clamp((b - d) / a, 0.0, 1.0);
        }
    }

    data.calculate(p0 + u*s, geom0->radius, q0 + v*t, geom1->radius);
    return data.distance;
}

double calculateDistance( const CollisionGeomSharedPtr& geom0,
                            const CollisionGeomSharedPtr& geom1,
                            CollisionData& data)
{
    if (geom0->getType() == CollisionGeom::SPHERE && geom1->getType() == CollisionGeom::SPHERE) {
        auto sph0 = std::static_pointer_cast<CollisionSphere>(geom0);
        auto sph1 = std::static_pointer_cast<CollisionSphere>(geom1);
        data.calculate(sph0->p_g, sph0->radius, sph1->p_g, sph1->radius);
        return data.distance;
    }
    else if (geom0->getType() == CollisionGeom::SPHERE && geom1->getType() == CollisionGeom::CAPSULE) {
        return sphereCapsuleDistance(std::static_pointer_cast<CollisionSphere>(geom0),
                                        std::static_pointer_cast<CollisionCapsule>(geom1), data);
    }
    else if (geom0->getType() == CollisionGeom::CAPSULE && geom1->getType() == CollisionGeom::SPHERE) {
        sphereCapsuleDistance(std::static_pointer_cast<CollisionSphere>(geom1), 
                                        std::static_pointer_cast<CollisionCapsule>(geom0), data);
        data.swapPoints();
        return data.distance;
    }
    else if (geom0->getType() == CollisionGeom::CAPSULE && geom1->getType() == CollisionGeom::CAPSULE) {
        return capsulesDistance(std::static_pointer_cast<CollisionCapsule>(geom0),
                                std::static_pointer_cast<CollisionCapsule>(geom1), data);
    }
    // else

    std::stringstream ss;
    ss << "ERROR: could not calculate distance shapes of types: " << geom0->getType()
                                                                << " and " << geom1->getType();
    std::cout << ss.str() << std::endl;
    throw std::runtime_error(ss.str());
    return 0;
}

void WutVelmaModel::calculateSelfCollisions(std::vector<CollisionData>& col_data) {
    if (col_data.size() != getCollisionPairsCount()) {
        std::stringstream ss;
        ss << "ERROR: wrong size of col_data buffer: " << col_data.size()
                                                    << ", should be " << getCollisionPairsCount();
        std::cout << ss.str() << std::endl;
        throw std::runtime_error(ss.str());
    }
    // TODO: check if this method is RT-safe

    // First, update keypoints of the collision geometries
    for (size_t i = 0; i < collision_link_geoms_.size(); ++i) {
        auto link_idx = collision_link_indices_[i];
        collision_link_geoms_[i]->updateReferenceFrame(fk_[link_idx]);
    }

    for (size_t i = 0; i < collision_checks_.size(); ++i) {
        auto col0_idx = collision_checks_[i][0];
        col_data[i].link0_idx = collision_checks_[i][1];
        auto col1_idx = collision_checks_[i][2];
        col_data[i].link1_idx = collision_checks_[i][3];
        
        calculateDistance(collision_link_geoms_[col0_idx],
                    collision_link_geoms_[col1_idx],
                    col_data[i]);
    }
}

const std::vector<CollisionGeomSharedPtr>& WutVelmaModel::getCollisionGeoms() const {
    return collision_link_geoms_;
}

void WutVelmaModel::setJointPosition(const VVector& joint_position) {
    // RT-safe
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
    // TODO: check if this method is RT-safe

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
    // TODO: check if this method is RT-safe

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
    // TODO: check if this method is RT-safe

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
    // TODO: check if this method is RT-safe

    pinocchio::forwardKinematics(p_model_, *p_data_, *p_q_);
    pinocchio::updateFramePlacements(p_model_, *p_data_);
    for (size_t i = 0; i < fk_links_id_.size(); ++i) {
        const pinocchio::SE3 & T = p_data_->oMf[fk_links_id_[i]];
        fk_[i].linear().noalias() = T.rotation();
        fk_[i].translation().noalias() = T.translation();
    }
}

const std::vector<Eigen::Affine3d>& WutVelmaModel::getFk() const {
    // RT-safe
    return fk_;
}

// TODO: parse SRDF, create ACM
