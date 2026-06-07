// TODO: license

#pragma once

#include <Eigen/Geometry>
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

class WutVelmaModel {
public:

    WutVelmaModel(const std::string& urdf_xml,
        const std::vector<std::string> & joint_names,
        const std::vector<std::string> & link_names);

    void setJointPosition(const VVector& joint_position);

    // [[nodiscard]]
    void calculateMassMatrix(VMatrix& mass_matrix);

    void calculateJacobian(Jacobian& jacobian);

    void calculateTcpFk(Eigen::Affine3d& p1, Eigen::Affine3d& p2);

    void calculateFk();
    const std::vector<Eigen::Affine3d>& getFk() const;

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
};
