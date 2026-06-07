// TODO: license

#ifndef WUT_VELMA_TYPES_HPP_
#define WUT_VELMA_TYPES_HPP_

#include <Eigen/Dense>

// TODO: add namespace
// TODO: rename types

// Active joints (controlled):
constexpr int Vjoints = 15;
typedef Eigen::Matrix<double, Vjoints, 1>  VVector;
typedef Eigen::Matrix<double, Vjoints, Vjoints>  VMatrix;

// All joints:
// TODO: verify number of all joints:
constexpr int Valljoints = 33;
typedef Eigen::Matrix<double, Valljoints, 1>  VallVector;

// End effectors
constexpr int EEcount = 2;
constexpr int EEidxL = 0;
constexpr int EEidxR = 1;

typedef Eigen::Matrix<double, 7, 1>  PoseVector7;
typedef Eigen::Matrix<double, 6, 1>  TwistVector6;

typedef Eigen::Matrix<double, EEcount * 6, Vjoints> Matrix6E_V;
typedef Eigen::Matrix<double, 6, Vjoints> Matrix6_V;
typedef Eigen::Matrix<double, EEcount * 6, Vjoints> Jacobian;
typedef Eigen::Matrix<double, Vjoints, EEcount * 6> JacobianT;
typedef Eigen::Matrix<double, EEcount * 6, 1> Vector2E;
typedef Eigen::Matrix<double, EEcount * 6, EEcount * 6> Matrix2E;


// constexpr std::array<std::string_view, Vjoints> Vjoint_names = {
//     "torso_0_joint",
//     "right_arm_0_joint",
//     "right_arm_1_joint",
//     "right_arm_2_joint",
//     "right_arm_3_joint",
//     "right_arm_4_joint",
//     "right_arm_5_joint",
//     "right_arm_6_joint",
//     "left_arm_0_joint",
//     "left_arm_1_joint",
//     "left_arm_2_joint",
//     "left_arm_3_joint",
//     "left_arm_4_joint",
//     "left_arm_5_joint",
//     "left_arm_6_joint",
// };

// TODO: other joints:
// "head_pan_joint"
// "head_tilt_joint"
// "rightKeepUprightJoint0"
// "rightKeepUprightJoint1"
// "rightFtSensorJoint"
// "right_HandFingerOneKnuckleOneJoint"
// "right_HandFingerOneKnuckleTwoJoint"
// "right_HandFingerOneKnuckleThreeJoint"
// "right_HandFingerThreeKnuckleTwoJoint"
// "right_HandFingerThreeKnuckleThreeJoint"
// "right_HandFingerTwoKnuckleOneJoint"
// "right_HandFingerTwoKnuckleTwoJoint"
// "right_HandFingerTwoKnuckleThreeJoint"
// "leftKeepUprightJoint0"
// "leftKeepUprightJoint1"
// "leftFtSensorJoint"
// "left_HandFingerOneKnuckleOneJoint"
// "left_HandFingerOneKnuckleTwoJoint"
// "left_HandFingerOneKnuckleThreeJoint"
// "left_HandFingerThreeKnuckleTwoJoint"
// "left_HandFingerThreeKnuckleThreeJoint"
// "left_HandFingerTwoKnuckleOneJoint"
// "left_HandFingerTwoKnuckleTwoJoint"
// "left_HandFingerTwoKnuckleThreeJoint"


#endif  // WUT_VELMA_TYPES_HPP_
