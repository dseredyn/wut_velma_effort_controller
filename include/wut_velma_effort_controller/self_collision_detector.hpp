// TODO: license

#ifndef COMMON_CORE_CS_COMPONENTS_COLLISION_DETECTOR_H__
#define COMMON_CORE_CS_COMPONENTS_COLLISION_DETECTOR_H__

#include <Eigen/Dense>
#include <visualization_msgs/msg/marker_array.hpp>

#include "wut_velma_effort_controller/wut_velma_types.hpp"

// #include <sstream>

// #include "eigen_conversions/eigen_msg.h"

// #include <math.h>

// #include <collision_convex_model/collision_convex_model.h>
// #include <kin_dyn_model/kin_model.h>
// #include "planer_utils/activation_function.h"

class SelfCollisionDetector {
public:
    SelfCollisionDetector();
    void dbgGetCollisionShapes(visualization_msgs::msg::MarkerArray & out_m_array);

    void calculate(const VallVector & all_q_, const VallVector & all_dq_,
                bool allow_hands_col, bool calculate_forces, const VMatrix & mInv_in_,
                const VVector & Nt_in_, VVector & t_out_, VMatrix & N_out_);

protected:

    //     VectorN q_in_;
//     RTT::InputPort<VectorN > port_q_in_;

//     VectorN dq_in_;
//     RTT::InputPort<VectorN > port_dq_in_;
};

// template<unsigned int N, unsigned int M, unsigned int Npairs >
// class CollisionDetectorComponent: public RTT::TaskContext {
// public:
//     explicit CollisionDetectorComponent(const std::string &name);

//     bool configureHook();

//     bool startHook();

//     void stopHook();

//     void updateHook();

//     std::string getDiag();

// private:

//     typedef Eigen::Matrix<double, N, 1>  VectorN;
//     typedef Eigen::Matrix<double, M, 1>  VectorM;
//     typedef Eigen::Matrix<double, M, M>  MatrixMM;
//     typedef boost::array<self_collision::CollisionInfo, Npairs > CollisionList;

//     // OROCOS ports
//     VectorN q_in_;
//     RTT::InputPort<VectorN > port_q_in_;

//     VectorN dq_in_;
//     RTT::InputPort<VectorN > port_dq_in_;

//     VectorM articulated_dq_in_;

//     MatrixMM mInv_in_;
//     RTT::InputPort<MatrixMM > port_mInv_in_;

//     CollisionList col_out_;
//     RTT::OutputPort<CollisionList > port_col_out_;

//     RTT::InputPort<int > port_allow_hands_col_;

//     // collision torques
//     VectorM t_out_;
//     RTT::OutputPort<VectorM > port_t_out_;

//     // collision task null space
//     VectorM Nt_in_;
//     RTT::InputPort<VectorM > port_Nt_in_;

//     MatrixMM N_out_;

//     // OROCOS properties
//     double activation_dist_;
//     std::string robot_description_;
//     std::string robot_description_semantic_;
//     std::string robot_description_semantic_no_hands_;
//     std::vector<std::string > joint_names_;
//     std::vector<std::string > articulated_joint_names_;
//     bool calculate_forces_;
//     double Fmax_;
//     std::vector<double > damping_factors_;

//     std::array<int, M> map_idx_q_nr_;
//     std::vector<std::pair<int, int> > map_ign_idx_q_nr_;

//     std::shared_ptr<ActivationFunction > af_;

//     std::vector<self_collision::CollisionInfo > col_;
//     Eigen::VectorXd q_;

//     boost::shared_ptr<self_collision::CollisionModel> col_model_;
//     boost::shared_ptr<KinematicModel> kin_model_;
//     std::vector<KDL::Frame > links_fk_;
//     std::vector<std::string > link_names_vec_;

//     int collisions_;

//     bool in_collision_;

//     KinematicModel::Jacobian jac1_, jac2_;
//     KinematicModel::Jacobian jac1_lin_, jac2_lin_;

//     int diag_l_idx;

//     int full_srdf_id_;
//     int no_hands_srdf_id_;
//     bool allow_hands_col_;
// };

// template<unsigned int N, unsigned int M, unsigned int Npairs >
// CollisionDetectorComponent<N, M, Npairs >::CollisionDetectorComponent(const std::string &name)
//     : TaskContext(name, PreOperational)
//     , port_q_in_("q_INPORT")
//     , port_dq_in_("dq_INPORT")
//     , port_col_out_("col_OUTPORT")
//     , port_allow_hands_col_("allow_hands_col_INPORT")
//     , activation_dist_(-1.0)
//     , collisions_(0)
//     , in_collision_(false)
//     , jac1_(6, N)
//     , jac2_(6, N)
//     , jac1_lin_(3, N)
//     , jac2_lin_(3, N)
//     , calculate_forces_(false)
//     , Fmax_(-1)
//     , diag_l_idx(0)
//     , allow_hands_col_(false)
// {
//     this->ports()->addPort(port_q_in_);
//     this->ports()->addPort(port_dq_in_);
//     this->ports()->addPort(port_col_out_);
//     this->ports()->addPort(port_allow_hands_col_);

//     this->addProperty("activation_dist", activation_dist_);
//     this->addProperty("robot_description", robot_description_);
//     this->addProperty("robot_description_semantic", robot_description_semantic_);
//     this->addProperty("robot_description_semantic_no_hands", robot_description_semantic_no_hands_);
//     this->addProperty("joint_names", joint_names_);
//     this->addProperty("articulated_joint_names", articulated_joint_names_);
//     this->addProperty("calculate_forces", calculate_forces_);
//     this->addProperty("Fmax", Fmax_);
//     this->addProperty("damping_factors", damping_factors_);

//     this->addOperation("getDiag", &CollisionDetectorComponent<N, M, Npairs >::getDiag, this, RTT::ClientThread);
//     this->addAttribute("inCollision", in_collision_);

//     col_.resize(Npairs);

//     for (int i = 0; i < Npairs; ++i) {
//         col_[i].link1_idx = -1;
//         col_out_[i].link1_idx = -1;
//     }
// }

// template<unsigned int N, unsigned int M, unsigned int Npairs >
// std::string CollisionDetectorComponent<N, M, Npairs >::getDiag() {
//     std::array<self_collision::CollisionInfo, Npairs > col;
//     std::ostringstream strs;

//     for (int i = 0; i < Npairs; ++i) {
//         col[i] = col_out_[i];
//     }

//     strs << "<cd col_count=\"" << collisions_ << "\">";
//     for (int i = 0; i < Npairs; ++i) {
//         if (col[i].link1_idx != -1) {
//             strs << "<c ";
//             strs << "i1=\"" << col[i].link1_idx << "\" ";
//             strs << "i2=\"" << col[i].link2_idx << "\" ";
//             strs << "p1x=\"" << col[i].p1_B.x() << "\" ";
//             strs << "p1y=\"" << col[i].p1_B.y() << "\" ";
//             strs << "p1z=\"" << col[i].p1_B.z() << "\" ";
//             strs << "p2x=\"" << col[i].p2_B.x() << "\" ";
//             strs << "p2y=\"" << col[i].p2_B.y() << "\" ";
//             strs << "p2z=\"" << col[i].p2_B.z() << "\" ";
//             strs << "d=\"" << col[i].dist << "\" ";
//             strs << "n1x=\"" << col[i].n1_B.x() << "\" ";
//             strs << "n1y=\"" << col[i].n1_B.y() << "\" ";
//             strs << "n1z=\"" << col[i].n1_B.z() << "\" ";
//             strs << "n2x=\"" << col[i].n2_B.x() << "\" ";
//             strs << "n2y=\"" << col[i].n2_B.y() << "\" ";
//             strs << "n2z=\"" << col[i].n2_B.z() << "\" ";
//             strs << "/>";
//         }
//     }

//     const self_collision::Link::VecPtrCollision& vec_col = col_model_->getLinkCollisionArray(diag_l_idx);
//     if (vec_col.size() > 0) {
//         strs << "<l idx=\"" << diag_l_idx << "\" name=\"" << col_model_->getLinkName(diag_l_idx) << "\">";
//         for (int j = 0; j < vec_col.size(); ++j) {
//             strs << "<g ";
//             strs << "x=\"" << vec_col[j]->origin.p.x() << "\" ";
//             strs << "y=\"" << vec_col[j]->origin.p.y() << "\" ";
//             strs << "z=\"" << vec_col[j]->origin.p.z() << "\" ";
//             double qx, qy, qz, qw;
//             vec_col[j]->origin.M.GetQuaternion(qx,qy,qz,qw);
//             strs << "qx=\"" << qx << "\" ";
//             strs << "qy=\"" << qy << "\" ";
//             strs << "qz=\"" << qz << "\" ";
//             strs << "qw=\"" << qw << "\" ";
            
//             int type = vec_col[j]->geometry->getType();
//             switch (type) {
//             case self_collision::Geometry::UNDEFINED:
//                 {
//                     strs << "type=\"UNDEFINED\" ";
//                     break;
//                 }
//             case self_collision::Geometry::CAPSULE:
//                 {
//                     strs << "type=\"CAPSULE\" ";
//                     boost::shared_ptr<self_collision::Capsule > ob = boost::dynamic_pointer_cast<self_collision::Capsule >(vec_col[j]->geometry);
//                     strs << "r=\"" << ob->getRadius() << "\" ";
//                     strs << "l=\"" << ob->getLength() << "\" ";
//                     break;
//                 }
//             case self_collision::Geometry::CONVEX:
//                 strs << "type=\"CONVEX\" ";
//                 break;
//             case self_collision::Geometry::SPHERE:
//                 {
//                     strs << "type=\"SPHERE\" ";
//                     boost::shared_ptr<self_collision::Sphere > ob = boost::dynamic_pointer_cast<self_collision::Sphere >(vec_col[j]->geometry);
//                     strs << "r=\"" << ob->getRadius() << "\" ";
//                     break;
//                 }
//             case self_collision::Geometry::TRIANGLE:
//                 strs << "type=\"TRIANGLE\" ";
//                 break;
//             case self_collision::Geometry::OCTOMAP:
//                 strs << "type=\"OCTOMAP\" ";
//                 break;
//             default:
//                 strs << "type=\"ERROR\" ";
//             }
//             strs << "/>";
//         }
//         strs << "</l>";
//     }

//     diag_l_idx = (diag_l_idx + 1) % col_model_->getLinksCount();

//     strs << "</cd>";

//     return strs.str();
// }

// template<unsigned int N, unsigned int M, unsigned int Npairs >
// bool CollisionDetectorComponent<N, M, Npairs >::configureHook() {
//     Logger::In in("CollisionDetectorComponent::configureHook");

//     // Get the rosparam service requester
//     boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this->getProvider<rtt_rosparam::ROSParam>("rosparam");

//       // Get the parameters
//     if(!rosparam) {
//         Logger::log() << Logger::Error << "Could not get ROS parameters from rtt_rosparam" << Logger::endl;
//         return false;
//     }

//     // Get the ROS parameter "/robot_description"
//     if (!rosparam->getAbsolute("robot_description")) {
//         Logger::log() << Logger::Error << "could not read ROS parameter \'robot_description\'" << Logger::endl;
//         return false;
//     }

//     if (!rosparam->getAbsolute("robot_description_semantic")) {
//         Logger::log() << Logger::Error << "could not read ROS parameter \'robot_description_semantic\'" << Logger::endl;
//         return false;
//     }

//     if (!rosparam->getAbsolute("robot_description_semantic_no_hands")) {
//         Logger::log() << Logger::Error << "could not read ROS parameter \'robot_description_semantic_no_hands\'" << Logger::endl;
//         return false;
//     }

// /*
//     RTT::OperationCaller<bool()> rosparam_getAbsolute = rosparam->getOperation("getAll");
//     if (!tc_rosparam_getAll.ready()) {
//         Logger::log() << Logger::Error << "could not get ROS parameter getAll operation for " << tc->getName() << Logger::endl;
//         return false;
//     }
//     if (!tc_rosparam_getAll()) {
//         Logger::log() << Logger::Warning << "could not read ROS parameters for " << tc->getName() << Logger::endl;
// //        return false;     // TODO: this IS an error
//     }
// */

//     if (joint_names_.size() != N) {
//         Logger::log() << Logger::Error << "ROS parameter \'joint_names\' has wrong size: " << joint_names_.size()
//             << ", should be: " << N << Logger::endl;
//         return false;
//     }

//     if (articulated_joint_names_.size() != M) {
//         Logger::log() << Logger::Error << "ROS parameter \'articulated_joint_names\' has wrong size: " << articulated_joint_names_.size()
//             << ", should be: " << M << Logger::endl;
//         return false;
//     }

//     for (int i = 0; i < M; ++i) {
//         bool found = false;
//         for (int j = 0; j < N; ++j) {
//             if (joint_names_[j] == articulated_joint_names_[i]) {
//                 found = true;
//                 break;
//             }
//         }
//         if (!found) {
//             Logger::log() << Logger::Error << "The set 'articulated_joint_names' must be a subset of 'joint_names'. Joint name '" << articulated_joint_names_[i]
//                 << "' is missing in 'joint_names'." << Logger::endl;
//             return false;
//         }
//     }

//     if (calculate_forces_) {
//         this->ports()->addPort("mInv_INPORT", port_mInv_in_);
//         this->ports()->addPort("Nt_INPORT", port_Nt_in_);
//         this->ports()->addPort("t_OUTPORT", port_t_out_);

//         if (Fmax_ <= 0.0) {
//             Logger::log() << Logger::Error << "Property \'Fmax\' is not set" << Logger::endl;
//             return false;
//         }

//         if (damping_factors_.size() != articulated_joint_names_.size()) {
//             Logger::log() << Logger::Error << "ROS parameter \'damping_factors\' has wrong size: " << damping_factors_.size()
//                 << ", should be: " << articulated_joint_names_.size() << Logger::endl;
//             return false;
//         }
//     }

//     kin_model_.reset( new KinematicModel(robot_description_, articulated_joint_names_) );

//     std::vector<std::string > ign_joint_name_vec;
//     kin_model_->getIgnoredJointsNameVector(ign_joint_name_vec);
//     for (int i = 0; i < ign_joint_name_vec.size(); ++i) {
//         const std::string &joint_name = ign_joint_name_vec[i];
//         bool found = false;
//         for (int j = 0; j < N; ++j) {
//             if (joint_name == joint_names_[j]) {
//                 map_ign_idx_q_nr_.push_back(std::pair<int, int >(i, j));
//                 found = true;
//                 break;
//             }
//         }
//         if (!found) {
//             Logger::log() << Logger::Info << "Could not find ignored joint '" << joint_name << "' in all joints list (probably it is ok)" << Logger::endl;
//             //return false;
//         }
//     }

//     for (int i = 0; i < N; ++i) {
//         bool articulated = false;
//         for (int j = 0; j < M; ++j) {
//             if (joint_names_[i] == articulated_joint_names_[j]) {
//                 articulated = true;
//                 map_idx_q_nr_[j] = i;
//                 break;
//             }
//         }
//         if (!articulated) {
//         }
//     }



//     q_.resize(M);

//     col_model_ = self_collision::CollisionModel::parseURDF(robot_description_);
//     full_srdf_id_ = col_model_->parseSRDF(robot_description_semantic_);
//     no_hands_srdf_id_ = col_model_->parseSRDF(robot_description_semantic_no_hands_);
//     col_model_->generateCollisionPairs();

//     links_fk_.resize(col_model_->getLinksCount());


//     if (activation_dist_ <= 0.0) {
//         Logger::log() << Logger::Error << "Property \'activation_dist\' is not set" << Logger::endl;
//         return false;
//     }

//     af_.reset(new ActivationFunction(0.2 * activation_dist_, 4.0 / activation_dist_));

//     link_names_vec_.clear();
//     for (int l_idx = 0; l_idx < col_model_->getLinksCount(); l_idx++) {
//         link_names_vec_.push_back(col_model_->getLinkName(l_idx));
//     }

//     return true;
// }

// template<unsigned int N, unsigned int M, unsigned int Npairs >
// bool CollisionDetectorComponent<N, M, Npairs >::startHook() {
//     collisions_ = 0;
//     in_collision_ = false;
//     return true;
// }

// template<unsigned int N, unsigned int M, unsigned int Npairs >
// void CollisionDetectorComponent<N, M, Npairs >::stopHook() {
//     collisions_ = 0;
//     in_collision_ = false;
// }

// template<unsigned int N, unsigned int M, unsigned int Npairs >
// void CollisionDetectorComponent<N, M, Npairs >::updateHook() {
//     //
//     // read HW status
//     //
//     if (port_q_in_.read(q_in_) != RTT::NewData) {
//         //Logger::In in("CollisionDetectorComponent::updateHook");
//         //Logger::log() << Logger::Error << "no data on port " << port_q_in_.getName() << Logger::endl;
//         error();
//         return;
//     }

//     if (port_dq_in_.read(dq_in_) != RTT::NewData) {
//         //Logger::In in("CollisionDetectorComponent::updateHook");
//         //Logger::log() << Logger::Error << "no data on port " << port_dq_in_.getName() << Logger::endl;
//         error();
//         return;
//     }

//     int allow_hands_col;
//     if (port_allow_hands_col_.read(allow_hands_col) == RTT::NewData) {
//         if (allow_hands_col == 0) {
//             allow_hands_col_ = false;
//         }
//         else {
//             allow_hands_col_ = true;
//         }
//     }

//     if (calculate_forces_) {
//         if (port_mInv_in_.read(mInv_in_) != RTT::NewData) {
//             //Logger::In in("CollisionDetectorComponent::updateHook");
//             //Logger::log() << Logger::Error << "no data on port " << port_mInv_in_.getName() << Logger::endl;
//             error();
//             return;
//         }
//         Nt_in_.setZero();
//         if (port_Nt_in_.read(Nt_in_) != RTT::NewData) {
//         }
//         t_out_.setZero();
//         N_out_.setIdentity();
//     }

//     for (int idx = 0; idx < map_ign_idx_q_nr_.size(); ++idx) {
//         kin_model_->setIgnoredJointValue(map_ign_idx_q_nr_[idx].first, q_in_(map_ign_idx_q_nr_[idx].second));
//     }

//     for (int i = 0; i < M; ++i) {
//         q_(i) = q_in_(map_idx_q_nr_[i]);
//         articulated_dq_in_(i) = dq_in_(map_idx_q_nr_[i]);
//     }

//     kin_model_->calculateFkAll(q_);

//     // calculate forward kinematics for all links
//     for (int l_idx = 0; l_idx < col_model_->getLinksCount(); l_idx++) {
//         links_fk_[l_idx] = kin_model_->getFrame(col_model_->getLinkName(l_idx));
//     }

//     if (allow_hands_col_) {
//         getCollisionPairsNoAlloc(col_model_, no_hands_srdf_id_, links_fk_, activation_dist_, col_);
//     }
//     else {
//         getCollisionPairsNoAlloc(col_model_, full_srdf_id_, links_fk_, activation_dist_, col_);
//     }

//     int collisions = 0;
//     int col_out_idx = 0;

//     for (int i = 0; i < Npairs; ++i) {
//         if (col_[i].link1_idx == -1) {
//             break;
//         }

//         const KDL::Frame &T_B_L1 = links_fk_[col_[i].link1_idx];
//         const std::string &link1_name = link_names_vec_[col_[i].link1_idx];
//         const std::string &link2_name = link_names_vec_[col_[i].link2_idx];

//         KDL::Frame T_L1_B = T_B_L1.Inverse();
//         const KDL::Frame &T_B_L2 = links_fk_[col_[i].link2_idx];
//         KDL::Frame T_L2_B = T_B_L2.Inverse();
//         KDL::Vector p1_L1 = T_L1_B * col_[i].p1_B;
//         KDL::Vector p2_L2 = T_L2_B * col_[i].p2_B;
//         KDL::Vector n1_L1 = KDL::Frame(T_L1_B.M) * col_[i].n1_B;
//         KDL::Vector n2_L2 = KDL::Frame(T_L2_B.M) * col_[i].n2_B;

//         kin_model_->getJacobiansForPairX(jac1_, jac2_, link1_name, p1_L1, link2_name, p2_L2, q_);

//         // the mapping between motions along contact normal and the Cartesian coordinates
//         KDL::Vector e1 = KDL::Frame(T_B_L1.M) * n1_L1;
//         KDL::Vector e2 = KDL::Frame(T_B_L2.M) * n2_L2;
//         Eigen::Matrix<double, 3, 1 > Jd1, Jd2;
//         for (int i = 0; i < 3; i++) {
//             Jd1[i] = e1[i];
//             Jd2[i] = e2[i];
//         }

//         for (int q_idx = 0; q_idx < N; q_idx++) {
//             for (int row_idx = 0; row_idx < 3; row_idx++) {
//                 jac1_lin_(row_idx, q_idx) = jac1_(row_idx, q_idx);
//                 jac2_lin_(row_idx, q_idx) = jac2_(row_idx, q_idx);
//             }
//         }

//         //KinematicModel::Jacobian
//         Eigen::Matrix<double, 1, M > Jcol1 = Jd1.transpose() * jac1_lin_;
//         //KinematicModel::Jacobian
//         Eigen::Matrix<double, 1, M > Jcol2 = Jd2.transpose() * jac2_lin_;

//         //KinematicModel::Jacobian Jcol(1, N);
//         Eigen::Matrix<double, 1, M > Jcol;
//         for (int q_idx = 0; q_idx < N; q_idx++) {
//             Jcol(0, q_idx) = Jcol1(0, q_idx) + Jcol2(0, q_idx);
//         }

//         // calculate relative velocity between points (1 dof)
//         double ddij = (Jcol * articulated_dq_in_)(0,0);

//         // for collision visualization
//         col_out_[col_out_idx] = col_[i];
//         ++col_out_idx;

//         // for collision predicate
//         if (ddij > 0.01) {
//             ++collisions;
//         }

//         if (calculate_forces_) {
//             double depth = (activation_dist_ - col_[i].dist);

//             // repulsive force
//             double f = 0.0;
//             if (col_[i].dist <= activation_dist_) {
//                 f = depth / activation_dist_;
//             }
//             else {
//                 f = 0.0;
//             }

//             if (f > 1.0) {
//                 f = 1.0;
//             }

//             double Frep = Fmax_ * f * f;

//             double K = 2.0 * Fmax_ / (activation_dist_ * activation_dist_);

//             // calculate collision mass (1 dof)
//             double Mdij_inv = (Jcol * mInv_in_ * Jcol.transpose())(0,0);

//             double D = 2.0 * 0.7 * sqrt(Mdij_inv * K);  // sqrt(K/M)

//             double activation = 1.0 - af_->func_Ndes(col_[i].dist);
//             MatrixMM Ncol12;
//             Ncol12.setIdentity();
//             Ncol12 = Ncol12 - (Jcol.transpose() * activation * Jcol);

//             ddij = 0;
//             VectorM d_torque = Jcol.transpose() * (-Frep - D * ddij);
//             t_out_ += d_torque;
//             N_out_ = N_out_ * Ncol12;
//         }
//     }

//     if (calculate_forces_) {
//         t_out_ += N_out_ * Nt_in_;  // apply null space torque
//         // apply simple damping
//         for (int i = 0; i < M; ++i) {
//             t_out_[i] -= damping_factors_[i] * articulated_dq_in_[i];
//         }

//         port_t_out_.write(t_out_);
//     }

//     for (int i = col_out_idx; i < Npairs; ++i) {
//         col_out_[i].link1_idx = -1;
//         col_out_[i].link2_idx = -1;
//     }

//     collisions_ = collisions;

//     if (!calculate_forces_ && collisions_ > 0) {
//         in_collision_ = true;
//     }
//     else {
//         in_collision_ = false;
//     }
// }

#endif  // COMMON_CORE_CS_COMPONENTS_COLLISION_DETECTOR_H__

