// TODO: license

#pragma once

#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
#include <memory>
#include <optional>

#include <urdf/model.h>
#include <Eigen/Dense>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>

#include "wut_velma_effort_controller/wut_velma_types.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>

class DualTcpJacobiansNoAlloc;
typedef std::shared_ptr<DualTcpJacobiansNoAlloc> DualTcpJacobiansNoAllocSharedPtr;

class TwoTcpPoseNoAlloc;
typedef std::shared_ptr<TwoTcpPoseNoAlloc> TwoTcpPoseNoAllocSharedPtr;


class CollisionGeom {
public:
    enum Type {SPHERE=1, CAPSULE=2};
    virtual Type getType() const;
    virtual void updateReferenceFrame(const Eigen::Affine3d& f) = 0;
protected:
    CollisionGeom(Type type);
private:
    Type type_;
};
typedef std::shared_ptr<CollisionGeom> CollisionGeomSharedPtr;


class CollisionSphere : public CollisionGeom {
public:
    CollisionSphere();
    CollisionSphere(double radius, Eigen::Vector3d p);
    virtual void updateReferenceFrame(const Eigen::Affine3d& f) override;

    double radius;
    Eigen::Vector3d p;

    // Sphere center point expressed in the global frame
    Eigen::Vector3d p_g;
};
typedef std::shared_ptr<CollisionSphere> CollisionSphereSharedPtr;


class CollisionCapsule : public CollisionGeom {
public:
    CollisionCapsule();
    CollisionCapsule(double radius, Eigen::Vector3d p0, Eigen::Vector3d p1);
    virtual void updateReferenceFrame(const Eigen::Affine3d& f) override;

    double radius;
    double length;
    Eigen::Vector3d p0;
    Eigen::Vector3d p1;

    // Capsule points expressed in the global frame
    Eigen::Vector3d p0_g;
    Eigen::Vector3d p1_g;
};
typedef std::shared_ptr<CollisionCapsule> CollisionCapsuleSharedPtr;

class CollisionData {
public:
    CollisionData();
    CollisionData(size_t link0_idx, size_t link1_idx);
    void swapPoints();
    void calculate(const Eigen::Vector3d& c0, double r0, const Eigen::Vector3d& c1, double r1);
    size_t link0_idx;
    Eigen::Vector3d p0;
    size_t link1_idx;
    Eigen::Vector3d p1;
    double distance;
};


class WutVelmaModel {
public:

    WutVelmaModel(
        const std::string& urdf_xml,
        const std::vector<std::string> & joint_names,
        const std::vector<std::string> & link_names,
        const std::string& srdf_xml,
        double collision_distance,
        std::vector<std::string> collision_link_names,
        std::vector<CollisionGeomSharedPtr> collision_link_geoms,
        const std::vector<std::pair<std::string, std::string>>& collision_pairs);

    void setJointPosition(const VVector& joint_position);

    // [[nodiscard]]
    void calculateMassMatrix(VMatrix& mass_matrix);

    void calculateJacobian(Jacobian& jacobian);

    void calculateTcpFk(Eigen::Affine3d& p1, Eigen::Affine3d& p2);

    void calculateFk();
    const std::vector<Eigen::Affine3d>& getFk() const;

    void calculateSelfCollisions(std::vector<CollisionData>& col_data);

    // const std::vector<CollisionData>& getSelfCollisions() const;

    const std::vector<CollisionGeomSharedPtr>& getCollisionGeoms() const;

    size_t getCollisionPairsCount() const;

protected:
    // pinocchio
    pinocchio::Model p_model_;
    std::optional<pinocchio::Data> p_data_;
    std::optional<Eigen::VectorXd> p_q_;

    std::array<std::pair<int, int>, Vjoints > p2v_map_;

    // For FK
    std::vector<std::string> fk_links_;
    std::vector<int> fk_links_id_;
    std::vector<Eigen::Affine3d> fk_;

    DualTcpJacobiansNoAllocSharedPtr dual_tcp_jac_;
    TwoTcpPoseNoAllocSharedPtr two_tcp_pose_;

    // For self-collisions
    std::vector<CollisionGeomSharedPtr> collision_link_geoms_;
    std::vector<size_t> collision_link_indices_;

    std::vector<std::array<size_t, 4>> collision_checks_;
    // std::vector<CollisionData> collisions_;
};
