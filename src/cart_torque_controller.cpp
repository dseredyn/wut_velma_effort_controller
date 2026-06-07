// TODO: license

#include "wut_velma_effort_controller/cart_torque_controller.hpp"

#include <Eigen/src/Geometry/Transform.h>
#include <array>
#include <cstddef>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/string.hpp>

// tf debug
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/transform_broadcaster.hpp"

#include "wut_velma_effort_controller/wut_velma_model.hpp"

#include <optional>

#include "controller_interface/chainable_controller_interface.hpp"
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <stdexcept>
#include <string>

#include "wut_velma_effort_controller/cartesian_impedance.hpp"
#include "wut_velma_effort_controller/wut_velma_types.hpp"


namespace wut_velma_effort_controller
{

double phase = 0;
double init_z = 0;
bool gen_initialized = false;
int iteration = 0;

inline std::string get_urdf_from_topic_detached(
  const std::string & topic_name = "/robot_description",
  std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
{
    auto tmp_node = std::make_shared<rclcpp::Node>("urdf_fetcher");

    auto prom = std::make_shared<std::promise<std::string>>();
    auto fut  = prom->get_future();

    // żeby set_value poszło tylko raz
    auto done = std::make_shared<std::atomic_bool>(false);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    std::function<void(std_msgs::msg::String::ConstSharedPtr)> cb =
        [prom, done](std_msgs::msg::String::ConstSharedPtr msg)
        {
        if (!msg || msg->data.empty()) return;

        bool expected = false;
        if (!done->compare_exchange_strong(expected, true)) return; // już ustawione

        prom->set_value(msg->data);
        };

    auto sub = tmp_node->create_subscription<std_msgs::msg::String>(topic_name, qos, cb);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(tmp_node);

    auto rc = exec.spin_until_future_complete(fut, timeout);

    exec.remove_node(tmp_node);

    if (rc != rclcpp::FutureReturnCode::SUCCESS) {
        throw std::runtime_error("Timeout waiting for robot_description on topic: " + topic_name);
    }

    return fut.get();
}

class TfDebug
{
public:
    TfDebug(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
                                                    const std::vector<std::string>& link_names)
    : tf_broadcaster_(node)
    {
        for (const auto & link_name : link_names) {
            geometry_msgs::msg::TransformStamped t;
            t.header.frame_id = "world";
            t.child_frame_id = link_name;
            tcp_tf_msg_.push_back(t);
        }
    }

    void publish(const rclcpp::Time& time, const std::vector<Eigen::Affine3d>& frames)
    {
        
        // for (const auto & t : tcp_tf_msg_) {
        for (size_t i = 0; i < tcp_tf_msg_.size(); ++i) {
            geometry_msgs::msg::TransformStamped& t = tcp_tf_msg_[i];
            const Eigen::Affine3d& T = frames[i];

            t.header.stamp = time;

            t.transform.translation.x = T.translation().x();
            t.transform.translation.y = T.translation().y();
            t.transform.translation.z = T.translation().z();

            Eigen::Quaterniond q(T.rotation());
            q.normalize();

            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            tf_broadcaster_.sendTransform(t);
        }
    }

private:
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::vector<geometry_msgs::msg::TransformStamped> tcp_tf_msg_;

    std::vector<std::string> link_names_;
    std::string tf_parent_frame_ = "base_link";
    std::string tf_child_frame_ = "tcp";
};



// class CartTorqueController : public controller_interface::ChainableControllerInterface
// {
// public:
//     controller_interface::CallbackReturn on_init() override;

//     controller_interface::InterfaceConfiguration
//     command_interface_configuration() const override;

//     controller_interface::InterfaceConfiguration
//     state_interface_configuration() const override;

//     controller_interface::CallbackReturn on_configure(
//         const rclcpp_lifecycle::State & previous_state) override;

//     controller_interface::CallbackReturn on_activate(
//         const rclcpp_lifecycle::State & previous_state) override;

//     controller_interface::CallbackReturn on_deactivate(
//         const rclcpp_lifecycle::State & previous_state) override;

//     bool on_set_chained_mode(bool chained_mode) override;

//     controller_interface::return_type update_and_write_commands(
//         const rclcpp::Time & time,
//         const rclcpp::Duration & period) override;

// protected:
//     std::vector<hardware_interface::CommandInterface>
//     on_export_reference_interfaces() override;

//     controller_interface::return_type update_reference_from_subscribers(
//         const rclcpp::Time & time,
//         const rclcpp::Duration & period) override;

// private:
//     size_t ref_index(size_t joint_index, size_t ref_type_index) const
//     {
//         // ref_type_index:
//         // 0 -> position
//         // 1 -> velocity
//         // 2 -> acceleration
//         return joint_index * 3 + ref_type_index;
//     }
    
//     std::vector<std::string> joints_;

//     realtime_tools::RealtimeBuffer<std::vector<double>> q_des_buf_;
//     rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;

//     CartesianImpedance cimp_;
//     std::optional<WutVelmaModel> velma_model_;

//     std::vector<std::string> command_interface_names_;
//     std::vector<std::string> state_interface_names_;
//     std::vector<std::string> reference_interface_names_;

//     DualTcpJacobiansNoAllocSharedPtr dual_tcp_jac_;

//     bool chained_mode_ = false;
// };





controller_interface::CallbackReturn CartTorqueController::on_init()
{
    // Parameters
    // auto node = get_node();
    // node->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration
CartTorqueController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = command_interface_names_;
  return config;
}

controller_interface::InterfaceConfiguration
CartTorqueController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = state_interface_names_;
  return config;
}



// controller_interface::InterfaceConfiguration
// CartTorqueController::command_interface_configuration() const
// {
//     controller_interface::InterfaceConfiguration cfg;
//     cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
//     cfg.names.reserve(joints_.size());
//     for (const auto & j : joints_) {
//         cfg.names.push_back(j + "/" + hardware_interface::HW_IF_EFFORT);
//     }
//     return cfg;
// }

// controller_interface::InterfaceConfiguration
// CartTorqueController::state_interface_configuration() const
// {
//     controller_interface::InterfaceConfiguration cfg;
//     cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
//     cfg.names.reserve(joints_.size() * 2);
//     for (const auto & j : joints_) {
//         cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
//         cfg.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
//     }
//     return cfg;
// }

std::vector<hardware_interface::CommandInterface>
CartTorqueController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(reference_interface_names_.size());

  for (size_t i = 0; i < reference_interface_names_.size(); ++i)
  {
    interfaces.emplace_back(
      get_node()->get_name(),
      reference_interface_names_[i],
      &reference_interfaces_[i]);
  }

  return interfaces;
}

std::vector<std::string> CartTorqueController::make_cartesian_interface_names(
  const std::array<std::string, EEcount> & cartesian_names) const
{
    std::vector<std::string> result;
    for (const auto & name : cartesian_names) {
        result.push_back(name + "/pose.position.x");
        result.push_back(name + "/pose.position.y");
        result.push_back(name + "/pose.position.z");
        result.push_back(name + "/pose.orientation.x");
        result.push_back(name + "/pose.orientation.y");
        result.push_back(name + "/pose.orientation.z");
        result.push_back(name + "/pose.orientation.w");

        // cartesian_name + "/twist.linear.x",
        // cartesian_name + "/twist.linear.y",
        // cartesian_name + "/twist.linear.z",
        // cartesian_name + "/twist.angular.x",
        // cartesian_name + "/twist.angular.y",
        // cartesian_name + "/twist.angular.z",
        // cartesian_name + "/accel.linear.x",
        // cartesian_name + "/accel.linear.y",
        // cartesian_name + "/accel.linear.z",
        // cartesian_name + "/accel.angular.x",
        // cartesian_name + "/accel.angular.y",
        // cartesian_name + "/accel.angular.z"
    }
    return result;
}

controller_interface::CallbackReturn
CartTorqueController::on_configure(const rclcpp_lifecycle::State &)
{
    auto node = get_node();

#ifdef NDEBUG
  std::cout << "Release (NDEBUG defined)\n";
#else
  std::cout << "Debug (NDEBUG not defined)\n";
#endif

    std::string urdf_xml;

    try {
        urdf_xml = get_urdf_from_topic_detached("/robot_description", std::chrono::milliseconds(3000));
        RCLCPP_INFO(get_node()->get_logger(), "Got robot_description (%zu bytes).", urdf_xml.size());
        // dalej: parsowanie URDF / budowa modelu (Pinocchio/KDL/itd.)
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }


    joints_ = node->get_parameter("joints").as_string_array();

    if (joints_.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' is empty.");
        return controller_interface::CallbackReturn::ERROR;
    }

    if (Vjoints != joints_.size()) {
        RCLCPP_ERROR(node->get_logger(), "Wrong number of joints in controller, expected 15");
        return controller_interface::CallbackReturn::ERROR;
    }

    // Print all controlled joints
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        std::cout << "CartTorqueController joint " << i <<": \"" << joints_[i] << "\""
                                                                                    << std::endl;
    }

    std::cout << "Creating WutVelmaModel" << std::endl;

    // TODO: Check if all useful links are listed.
    std::vector<std::string> fk_link_names{
        "torso_base",
        "torso_link0",
        "head_tilt_link",
        "right_arm_1_link",
        "right_arm_2_link",
        "right_arm_3_link",
        "right_arm_4_link",
        "right_arm_5_link",
        "right_arm_6_link",
        "right_arm_7_link",
        "left_arm_1_link",
        "left_arm_2_link",
        "left_arm_3_link",
        "left_arm_4_link",
        "left_arm_5_link",
        "left_arm_6_link",
        "left_arm_7_link"};
    velma_model_.emplace(urdf_xml, joints_, fk_link_names);

    std::cout << "Created WutVelmaModel" << std::endl;

    // Set q_des = current position
    // q_des_buf_.writeFromNonRT(std::vector<double>(joints_.size(), 0.0));

    command_interface_names_.clear();
    state_interface_names_.clear();
    reference_interface_names_.clear();

    for (const auto & joint : joints_)
    {
        // Fizyczny command interface hardware:
        command_interface_names_.push_back(
        joint + "/" + hardware_interface::HW_IF_EFFORT);

        // Fizyczne state interfaces hardware:
        state_interface_names_.push_back(
        joint + "/" + hardware_interface::HW_IF_POSITION);
        state_interface_names_.push_back(
        joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }

    // Order is important: arm_l, arm_r
    std::array<std::string, EEcount> ee_names;
    ee_names[EEidxL] = "arm_l";
    ee_names[EEidxR] = "arm_r";
    reference_interface_names_ = make_cartesian_interface_names(ee_names);
                                        
    // for (size_t i = 0; i < EEcount; ++i) {
        
    //     // Wirtualne wejścia kontrolera:
    //     // arms_torso_effort/torso_0_joint/position
    //     // arms_torso_effort/torso_0_joint/velocity
    //     // arms_torso_effort/torso_0_joint/acceleration
    //     reference_interface_names_.push_back(
    //     joint + "/" + hardware_interface::HW_IF_POSITION);
    //     reference_interface_names_.push_back(
    //     joint + "/" + hardware_interface::HW_IF_VELOCITY);
    //     reference_interface_names_.push_back(
    //     joint + "/" + hardware_interface::HW_IF_ACCELERATION);
    // }

    reference_interfaces_.assign(
        reference_interface_names_.size(),
        std::numeric_limits<double>::quiet_NaN());

    RCLCPP_INFO(
        get_node()->get_logger(),
        "Configured chainable computed torque controller with %zu joints",
        joints_.size());

    // tf debug
    // std::vector<std::string> fk_link_names_tf_dbg;
    // for (const auto & link_name : fk_link_names) {
    //     fk_link_names_tf_dbg.push_back(link_name + "_dbg");
    // }
    // tf_debug_ = std::make_unique<TfDebug>(get_node(), fk_link_names_tf_dbg);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartTorqueController::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::fill(
    reference_interfaces_.begin(),
    reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartTorqueController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (auto & cmd : command_interfaces_)
  {
    cmd.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

bool CartTorqueController::on_set_chained_mode(bool chained_mode)
{
  chained_mode_ = chained_mode;

  // Jeżeli miałbyś własny subscriber, np. ~/commands,
  // to tutaj należy go ignorować/dezaktywować w chained_mode == true.
  //
  // Dokumentacja zaleca wyłączenie zewnętrznych wejść w chained mode,
  // żeby uniknąć dwóch źródeł komend naraz.

  return true;
}

controller_interface::return_type
CartTorqueController::update_reference_from_subscribers(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  // Ta metoda jest używana tylko wtedy, gdy kontroler NIE jest spięty
  // z kontrolerem nadrzędnym.
  //
  // W minimalnym wariancie nie obsługujemy pracy standalone,
  // więc nic nie robimy.

  return controller_interface::return_type::OK;
}

// controller_interface::CallbackReturn
// CartTorqueController::on_activate(const rclcpp_lifecycle::State &)
// {
//     auto node = get_node();

//     if (joints_.empty()) {
//         RCLCPP_ERROR(node->get_logger(), "on_activate: joints_ is empty.");
//         return controller_interface::CallbackReturn::ERROR;
//     }

//     // We expect 1 command interface (effort) for each joint
//     if (command_interfaces_.size() != joints_.size()) {
//         RCLCPP_ERROR(
//         node->get_logger(),
//         "on_activate: command_interfaces size mismatch. Expected %zu, got %zu",
//         joints_.size(), command_interfaces_.size());
//         return controller_interface::CallbackReturn::ERROR;
//     }

//     // We expect 2 state interfaces (position, velocity) for each joint
//     const size_t expected_state = joints_.size() * 2;
//     if (state_interfaces_.size() < expected_state) {
//         RCLCPP_ERROR(
//         node->get_logger(),
//         "on_activate: state_interfaces size mismatch. Expected at least %zu, got %zu",
//         expected_state, state_interfaces_.size());
//         return controller_interface::CallbackReturn::ERROR;
//     }

//     // 1) Set q_des = current position (hold current pose)
//     std::vector<double> q0(joints_.size(), 0.0);

//     for (size_t i = 0; i < joints_.size(); ++i) {
//         // indices: [q0, dq0, q1, dq1, ...]
//         const size_t q_idx = 2 * i + 0;

//         auto q_opt = state_interfaces_[q_idx].get_optional();
//         if (!q_opt) {
//             RCLCPP_WARN(
//                 node->get_logger(),
//                 "on_activate: missing position state for joint '%s' (idx=%zu). Using 0.0",
//                 joints_[i].c_str(), i);
//             q0[i] = 0.0;
//         } else {
//             q0[i] = *q_opt;
//         }
//     }

//     q_des_buf_.writeFromNonRT(q0);

//     // 2) Reset current effort commands
//     for (auto & ci : command_interfaces_) {
//         if (!ci.set_value(0.0)) {
//         return controller_interface::CallbackReturn::ERROR;
//         }
//     }

//     RCLCPP_INFO(node->get_logger(), "Controller activated: q_des initialized to current joint positions.");
//     return controller_interface::CallbackReturn::SUCCESS;
// }

// controller_interface::return_type
// CartTorqueController::update(const rclcpp::Time &, const rclcpp::Duration &)
// {
//     const auto * q_des = q_des_buf_.readFromRT();
//     if (!q_des || q_des->size() != joints_.size()) {
//         return controller_interface::return_type::ERROR;
//     }

//     if (!velma_model_) {
//         return controller_interface::return_type::ERROR;
//     }

//     VVector joint_position;
//     VVector joint_position_cmd;
//     VVector stiffness;
//     VVector joint_velocity;
//     VVector nullspace_torque_cmd;
//     VMatrix mass_matrix;
//     VVector out_joint_torque_command;

//     for (size_t i = 0; i < joints_.size(); ++i) {
//         auto q_opt = state_interfaces_[2*i + 0].get_optional();
//         auto dq_opt = state_interfaces_[2*i + 1].get_optional();

//         if (q_opt && dq_opt) {
//             joint_position[i] = *q_opt;
//             joint_velocity[i] = *dq_opt;
//         }
//         else {
//             return controller_interface::return_type::ERROR;
//         }

//         joint_position_cmd[i] = (*q_des)[i];
//     }

//     velma_model_->setJointPosition(joint_position);
//     velma_model_->calculateMassMatrix(mass_matrix);
//     // mass_matrix.setConstant(1.0);

//     // TODO: read 'stiffness'
//     stiffness.setConstant(50.0);

//     nullspace_torque_cmd.setZero();
//     out_joint_torque_command.setZero();

//     if (!jimp_.calculate(joint_position, joint_position_cmd, stiffness, joint_velocity, nullspace_torque_cmd, mass_matrix, out_joint_torque_command)) {
//         std::cout << "CartTorqueController: could not calculate joint impedance" << std::endl;
//         return controller_interface::return_type::ERROR;
//     }

//     // state_interfaces_ holds [q0, dq0, q1, dq1, ...] as in the state_interface_configuration
//     for (size_t i = 0; i < joints_.size(); ++i) {
//         if (!command_interfaces_[i].set_value(out_joint_torque_command[i])) {
//             return controller_interface::return_type::ERROR;
//         }

//         // auto q_opt = state_interfaces_[2*i + 0].get_optional();
//         // auto dq_opt = state_interfaces_[2*i + 1].get_optional();

//         // if (q_opt && dq_opt) {
//         //     const double q  = *q_opt;
//         //     const double dq = *dq_opt;

//         //     const double e  = (*q_des)[i] - q;
//         //     const double de = 0.0 - dq;

//         //     const double tau = kp_ * e + kd_ * de;  // PD -> effort

//         //     if (!command_interfaces_[i].set_value(tau)) {
//         //         return controller_interface::return_type::ERROR;
//         //     }
//         // }
//     }

//     return controller_interface::return_type::OK;
// }

controller_interface::return_type
CartTorqueController::update_and_write_commands(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
    // const auto * q_des = q_des_buf_.readFromRT();
    // if (!q_des || q_des->size() != joints_.size()) {
    //     return controller_interface::return_type::ERROR;
    // }

    if (!velma_model_) {
        return controller_interface::return_type::ERROR;
    }

    VVector joint_position;
    VVector joint_velocity;
    VVector nullspace_torque_cmd;
    VMatrix mass_matrix;
    VVector out_joint_torque_command;

    // Read commands from higher controller
    bool cmd_valid = true;
    std::array<Eigen::Affine3d, EEcount> cart_position_cmd;
    for (size_t i = 0; i < EEcount; ++i) {
        const double px = reference_interfaces_[i*7 + 0];
        const double py = reference_interfaces_[i*7 + 1];
        const double pz = reference_interfaces_[i*7 + 2];
        const double qx = reference_interfaces_[i*7 + 3];
        const double qy = reference_interfaces_[i*7 + 4];
        const double qz = reference_interfaces_[i*7 + 5];
        const double qw = reference_interfaces_[i*7 + 6];

        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)
                || !std::isfinite(qx) || !std::isfinite(qy)
                || !std::isfinite(qz) || !std::isfinite(qw))
        {
            cmd_valid = false;
            break;
        }
        Eigen::Quaterniond q(qw, qx, qy, qz);
        q.normalize();
        cart_position_cmd[i].translation() = Eigen::Vector3d(px, py, pz);
        cart_position_cmd[i].linear() = q.toRotationMatrix();
    }
    if (cmd_valid) {
        cart_position_cmd_ = cart_position_cmd;
    }

    // Read HW state
    for (size_t i = 0; i < joints_.size(); ++i) {
        double q, dq;
        if (state_interfaces_[2*i + 0].get_value(q) &&
                state_interfaces_[2*i + 1].get_value(dq)) {
            joint_position[i] = q;
            joint_velocity[i] = dq;
        }
        else {
            return controller_interface::return_type::OK;
        }
    }

    bool verbose = false;

    if (iteration < 2) {
        iteration++;
        verbose = true;
        // return controller_interface::return_type::OK;
    }

    if (verbose) std::cout << "joint_position: " << joint_position << std::endl;

    if (true) {
        velma_model_->setJointPosition(joint_position);

        velma_model_->calculateMassMatrix(mass_matrix);

        // TODO: read Kc, Dxi

        nullspace_torque_cmd.setZero();
        out_joint_torque_command.setZero();

        Jacobian jacobian;
        std::array<Eigen::Affine3d, EEcount> cart_position;
        Vector2E Kc;
        Vector2E Dxi;

        velma_model_->calculateJacobian(jacobian);

        velma_model_->calculateTcpFk(cart_position[0], cart_position[1]);

        if (verbose) std::cout << "left: " << cart_position[0].translation().transpose()
                    << ", " << cart_position[0].linear().transpose() << std::endl;
        if (verbose) std::cout << "right: " << cart_position[1].translation().transpose()
                    << ", " << cart_position[1].linear().transpose() << std::endl;

        // for debug
        if (!gen_initialized) {
            cart_position_cmd_ = cart_position;
            init_z = cart_position_cmd_[0].translation().z();
            gen_initialized = true;
            std::cout << "left: " << cart_position[0].translation().transpose()
                    << ", " << cart_position[0].linear().transpose() << std::endl;
            std::cout << "right: " << cart_position[1].translation().transpose()
                    << ", " << cart_position[1].linear().transpose() << std::endl;
            std::cout << "left_cmd: " << cart_position_cmd_[0].translation().transpose()
                    << ", " << cart_position_cmd_[0].linear().transpose() << std::endl;
            std::cout << "right_cmd: " << cart_position_cmd_[1].translation().transpose()
                    << ", " << cart_position_cmd_[1].linear().transpose() << std::endl;
        }

        // tf debug
        // if (tf_debug_ && publish_tf_every_n_updates_ > 0) {
        //     ++tf_publish_counter_;

        //     if (tf_publish_counter_ >= publish_tf_every_n_updates_) {
        //         tf_publish_counter_ = 0;

        //         velma_model_->calculateFk();
        //         const auto & fk = velma_model_->getFk();
        //         tf_debug_->publish(get_node()->now(), fk);
        //     }
        // }

        // phase += 0.001;
        // cart_position_cmd_[0].translation().z() = init_z + sin(phase) * 0.1;
        if (verbose) std::cout << "cmd z: " << cart_position_cmd_[0].translation().z() << std::endl;

        // TODO: Kc, Dxi
        for (size_t i = 0; i < 2; ++i) {
            Kc(i*6 + 0) = 50;
            Kc(i*6 + 1) = 50;
            Kc(i*6 + 2) = 50;
            Kc(i*6 + 3) = 20;
            Kc(i*6 + 4) = 20;
            Kc(i*6 + 5) = 20;
        }
        for (size_t i = 0; i < 12; ++i) {
            Dxi(i) = 0.7;
        }

        if (!cimp_.calculate(joint_velocity, mass_matrix, jacobian,
                                cart_position, cart_position_cmd_, Kc, Dxi,
                                nullspace_torque_cmd, out_joint_torque_command)) {
            if (verbose) std::cout << "CartTorqueController: could not calculate Cartesian impedance"
                                                                                    << std::endl;
            return controller_interface::return_type::ERROR;
        }
    }
    else {
        for (size_t i = 0; i < joints_.size(); ++i) {
            out_joint_torque_command[i] = 0.0;
        }
    }
    // state_interfaces_ holds [q0, dq0, q1, dq1, ...] as in the state_interface_configuration
    for (size_t i = 0; i < joints_.size(); ++i) {
        if (!command_interfaces_[i].set_value(out_joint_torque_command[i])) {
            return controller_interface::return_type::ERROR;
        }
    }
    return controller_interface::return_type::OK;
}


}  // namespace wut_velma_effort_controller

PLUGINLIB_EXPORT_CLASS(
  wut_velma_effort_controller::CartTorqueController,
  controller_interface::ChainableControllerInterface)
