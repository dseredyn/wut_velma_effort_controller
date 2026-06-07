// TODO: license

// #include <sstream>

// #include <rtt/Component.hpp>
// #include <rtt/Logger.hpp>
// #include <rtt/base/PortInterface.hpp>
// #include <rtt_rosparam/rosparam.h>

// #include "eigen_conversions/eigen_msg.h"

// #include <math.h>

// #include <collision_convex_model/collision_convex_model.h>
// #include <kin_dyn_model/kin_model.h>
// #include "planer_utils/activation_function.h"


#include <visualization_msgs/msg/marker_array.hpp>
#include "wut_velma_effort_controller/self_collision_detector.hpp"
#include "wut_velma_effort_controller/wut_velma_types.hpp"
#include <kdl/frames.hpp>
#include <vector>

// bool CollisionModel::getDistance(const Geometry *geom1, const KDL::Frame &tf1, const Geometry *geom2, const KDL::Frame &tf2, KDL::Vector &d1_out, KDL::Vector &d2_out, KDL::Vector &n1_out, KDL::Vector &n2_out, double d0, double &distance)
// {
//     // check broadphase distance
//     if ((tf1.p - tf2.p).Norm() > geom1->getBroadphaseRadius() + geom2->getBroadphaseRadius() + d0)
//     {
//         distance = d0 * 2.0;
//         return true;
//     }
//     if (geom1->getType() == Geometry::CAPSULE && geom2->getType() == Geometry::CAPSULE)
//     {
//         const fcl_2::Capsule *ob1 = static_cast<fcl_2::Capsule* >(geom1->shape.get());
//         const fcl_2::Capsule *ob2 = static_cast<fcl_2::Capsule* >(geom2->shape.get());

//         // capsules are shifted by length/2
//         double x1,y1,z1,w1, x2, y2, z2, w2;
//         KDL::Frame tf1_corrected = tf1 * KDL::Frame(KDL::Vector(0,0,-ob1->lz/2.0));
//         tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
//         KDL::Frame tf2_corrected = tf2 * KDL::Frame(KDL::Vector(0,0,-ob2->lz/2.0));
//         tf2_corrected.M.GetQuaternion(x2,y2,z2,w2);

//         // output variables
//         fcl_2::Vec3f p1;
//         fcl_2::Vec3f p2;
//         fcl_2::Vec3f n1;
//         fcl_2::Vec3f n2;
//         bool result = gjk_solver.shapeDistance(
//             *ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
//             *ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2_corrected.p.x(),tf2_corrected.p.y(),tf2_corrected.p.z())),
//              &distance, &p1, &p2, &n1, &n2);
//         // the output for two capsules is in global coordinates
//         d1_out = KDL::Vector(p1[0], p1[1], p1[2]);
//         d2_out = KDL::Vector(p2[0], p2[1], p2[2]);
//         n1_out = KDL::Vector(n1[0], n1[1], n1[2]);
//         n2_out = KDL::Vector(n2[0], n2[1], n2[2]);
//         return result;
//     }
//     else if (geom1->getType() == Geometry::SPHERE && geom2->getType() == Geometry::SPHERE)
//     {
//         const fcl_2::Sphere *ob1 = static_cast<fcl_2::Sphere* >(geom1->shape.get());
//         const fcl_2::Sphere *ob2 = static_cast<fcl_2::Sphere* >(geom2->shape.get());

//         double x1,y1,z1,w1, x2, y2, z2, w2;
//         KDL::Frame tf1_corrected = tf1;
//         tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
//         KDL::Frame tf2_corrected = tf2;
//         tf2_corrected.M.GetQuaternion(x2,y2,z2,w2);

//         // output variables
//         fcl_2::Vec3f p1;
//         fcl_2::Vec3f p2;
//         fcl_2::Vec3f n1;
//         fcl_2::Vec3f n2;
//         bool result = gjk_solver.shapeDistance(
//             *ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
//             *ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2_corrected.p.x(),tf2_corrected.p.y(),tf2_corrected.p.z())),
//              &distance, &p1, &p2, &n1, &n2);
//         // the output for two spheres is in global coordinates
//         d1_out = KDL::Vector(p1[0], p1[1], p1[2]);
//         d2_out = KDL::Vector(p2[0], p2[1], p2[2]);
//         n1_out = KDL::Vector(n1[0], n1[1], n1[2]);
//         n2_out = KDL::Vector(n2[0], n2[1], n2[2]);
//         return result;
//     }
//     else if (geom1->getType() == Geometry::CAPSULE && geom2->getType() == Geometry::SPHERE)
//     {
//         const fcl_2::Capsule *ob1 = static_cast<fcl_2::Capsule* >(geom1->shape.get());
//         const fcl_2::Sphere *ob2 = static_cast<fcl_2::Sphere* >(geom2->shape.get());

//         // capsules are shifted by length/2
//         double x1,y1,z1,w1, x2, y2, z2, w2;
//         KDL::Frame tf1_corrected = tf1;
//         tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
//         KDL::Frame tf2_corrected = tf2;
//         tf2_corrected.M.GetQuaternion(x2,y2,z2,w2);

//         // output variables
//         fcl_2::Vec3f p1;
//         fcl_2::Vec3f p2;
//         fcl_2::Vec3f n1;
//         fcl_2::Vec3f n2;
//         bool result = gjk_solver.shapeDistance(
//             *ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2_corrected.p.x(),tf2_corrected.p.y(),tf2_corrected.p.z())),
//             *ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
//              &distance, &p2, &p1, &n2, &n1);

//         // the output is in objects coordinate frames
//         d1_out = tf1_corrected * KDL::Vector(p1[0], p1[1], p1[2]);
//         d2_out = tf2_corrected * KDL::Vector(p2[0], p2[1], p2[2]);
//         n1_out = KDL::Vector(n1[0], n1[1], n1[2]);
//         n2_out = KDL::Vector(n2[0], n2[1], n2[2]);

//         return result;
//     }
//     else if (geom1->getType() == Geometry::SPHERE && geom2->getType() == Geometry::CAPSULE)
//     {
//         return CollisionModel::getDistance(geom2, tf2, geom1, tf1, d2_out, d1_out, n2_out, n1_out, d0, distance);
//     }
//     else
//     {
//         ROS_ERROR("not supported distance measure");
//     }
//     return false;
// }



// void getCollisionPairsNoAlloc(const boost::shared_ptr<self_collision::CollisionModel> &col_model,
//                     int srdf_id, const std::vector<KDL::Frame > &links_fk, double activation_dist,
//                                     std::vector<self_collision::CollisionInfo> &link_collisions) {
//         const int N = link_collisions.size();
//         if (N == 0) {
//             std::cout << "ERROR: getCollisionPairsNoAlloc: link_collisions.size() == 0" << std::endl;
//             return;
//         }

//         for (int i = 0; i < N; ++i) {
//             link_collisions[i].link1_idx = -1;
//         }

//         int col_count = 0;

//         // self collision
//         for (self_collision::CollisionModel::CollisionPairs::const_iterator it =
//                                     col_model->enabled_collisions[srdf_id].begin();
//                                         it != col_model->enabled_collisions[srdf_id].end(); it++) {
//             int link1_idx = it->first;
//             int link2_idx = it->second;
//             KDL::Frame T_B_L1 = links_fk[link1_idx];
//             KDL::Frame T_B_L2 = links_fk[link2_idx];

//             for (self_collision::Link::VecPtrCollision::const_iterator col1 = col_model->getLinkCollisionArray(link1_idx).begin(); col1 != col_model->getLinkCollisionArray(link1_idx).end(); col1++) {
//                 for (self_collision::Link::VecPtrCollision::const_iterator col2 = col_model->getLinkCollisionArray(link2_idx).begin(); col2 != col_model->getLinkCollisionArray(link2_idx).end(); col2++) {
//                     double dist = 0.0;
//                     KDL::Vector p1_B, p2_B, n1_B, n2_B;
//                     KDL::Frame T_B_C1 = T_B_L1 * (*col1)->origin;
//                     KDL::Frame T_B_C2 = T_B_L2 * (*col2)->origin;

//                     // TODO: handle dist < 0
//                     if (!self_collision::CollisionModel::getDistance((*col1)->geometry.get(), T_B_C1, (*col2)->geometry.get(), T_B_C2, p1_B, p2_B, n1_B, n2_B, activation_dist, dist)) {
//                         std::cout << "ERROR: getCollisionPairs: dist < 0" << std::endl;
//                     }

//                     if (dist < activation_dist) {
//                         self_collision::CollisionInfo col_info;
//                         col_info.link1_idx = link1_idx;
//                         col_info.link2_idx = link2_idx;
//                         col_info.dist = dist;
//                         col_info.n1_B = n1_B;
//                         col_info.n2_B = n2_B;
//                         col_info.p1_B = p1_B;
//                         col_info.p2_B = p2_B;

//                         for (int i = 0; i < N; ++i) {
//                             if (link_collisions[i].link1_idx == -1) {
//                                 link_collisions[i] = col_info;
//                                 ++col_count;
//                                 break;
//                             }
//                             else if (link_collisions[i].dist > col_info.dist) {
//                                 int max_idx = ((col_count > N)?N:col_count);
//                                 for (int j = max_idx-1; j > i; --j) {
//                                     link_collisions[j] = link_collisions[j-1];
//                                 }
//                                 link_collisions[i] = col_info;
//                                 ++col_count;
//                                 break;
//                             }
//                         }
//                     }
//                 }
//             }
//         }
// }


SelfCollisionDetector::SelfCollisionDetector()
{}

    // VPVector all_dq_;

void SelfCollisionDetector::dbgGetCollisionShapes(visualization_msgs::msg::MarkerArray & out_m_array) {
    
}

// N=33
// M=15
// Npairs=100
void SelfCollisionDetector::calculate(const VallVector & all_q_, const VallVector & all_dq_,
                bool allow_hands_col, bool calculate_forces, const VMatrix & mInv_in_,
                const VVector & Nt_in_, VVector & t_out_, VMatrix & N_out_) {

    if (calculate_forces) {
        t_out_.setZero();
        N_out_.setIdentity();
    }

    // TODO: FK
    std::vector<KDL::Frame> links_fk_;

//     // for (int idx = 0; idx < map_ign_idx_q_nr_.size(); ++idx) {
//     //     kin_model_->setIgnoredJointValue(map_ign_idx_q_nr_[idx].first, all_q_(map_ign_idx_q_nr_[idx].second));
//     // }

//     // for (int i = 0; i < M; ++i) {
//     //     q_(i) = all_q_(map_idx_q_nr_[i]);
//     //     articulated_dq_in_(i) = all_dq_(map_idx_q_nr_[i]);
//     // }

//     // kin_model_->calculateFkAll(q_);

//     // // calculate forward kinematics for all links
//     // for (int l_idx = 0; l_idx < col_model_->getLinksCount(); l_idx++) {
//     //     links_fk_[l_idx] = kin_model_->getFrame(col_model_->getLinkName(l_idx));
//     // }

//     if (allow_hands_col) {
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
}



// TODO: remove below this line:

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
//     VectorN all_q_;
//     RTT::InputPort<VectorN > port_all_q_;

//     VectorN all_dq_;
//     RTT::InputPort<VectorN > port_all_dq_;

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
//     , port_all_q_("q_INPORT")
//     , port_all_dq_("dq_INPORT")
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
//     this->ports()->addPort(port_all_q_);
//     this->ports()->addPort(port_all_dq_);
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

