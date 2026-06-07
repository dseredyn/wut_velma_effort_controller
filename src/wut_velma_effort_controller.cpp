// TODO: license

#include "wut_velma_effort_controller/wut_velma_effort_controller.hpp"

#include <cstddef>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

#include "wut_velma_effort_controller/wut_velma_model.hpp"

namespace wut_velma_effort_controller
{


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


controller_interface::CallbackReturn WutVelmaEffortController::on_init()
{
    // Parameters
    // auto node = get_node();
    // node->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration
WutVelmaEffortController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = command_interface_names_;
  return config;
}

controller_interface::InterfaceConfiguration
WutVelmaEffortController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = state_interface_names_;
  return config;
}



// controller_interface::InterfaceConfiguration
// WutVelmaEffortController::command_interface_configuration() const
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
// WutVelmaEffortController::state_interface_configuration() const
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
WutVelmaEffortController::on_export_reference_interfaces()
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



controller_interface::CallbackReturn
WutVelmaEffortController::on_configure(const rclcpp_lifecycle::State &)
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
        std::cout << "WutVelmaEffortController joint " << i <<": \"" << joints_[i] << "\""
                                                                                    << std::endl;
    }

    std::cout << "Creating WutVelmaModel" << std::endl;

    // TODO: fk links
    std::vector<std::string> links;
    velma_model_.emplace(urdf_xml, joints_, links);

    std::cout << "Created WutVelmaModel" << std::endl;

    // Set q_des = current position
    // q_des_buf_.writeFromNonRT(std::vector<double>(joints_.size(), 0.0));

    // // Subscription of desired position (Float64MultiArray of length = number of joints)
    // sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    //     "~/q_des", rclcpp::SystemDefaultsQoS(),
    //     [this](const std_msgs::msg::Float64MultiArray & msg)
    //     {
    //         if (msg.data.size() == joints_.size()) {
    //             q_des_buf_.writeFromNonRT(msg.data);
    //         }
    //     });

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

        // Wirtualne wejścia kontrolera:
        // arms_torso_effort/torso_0_joint/position
        // arms_torso_effort/torso_0_joint/velocity
        // arms_torso_effort/torso_0_joint/acceleration
        reference_interface_names_.push_back(
        joint + "/" + hardware_interface::HW_IF_POSITION);
        reference_interface_names_.push_back(
        joint + "/" + hardware_interface::HW_IF_VELOCITY);
        reference_interface_names_.push_back(
        joint + "/" + hardware_interface::HW_IF_ACCELERATION);
    }

    reference_interfaces_.assign(
        reference_interface_names_.size(),
        std::numeric_limits<double>::quiet_NaN());

    RCLCPP_INFO(
        get_node()->get_logger(),
        "Configured chainable computed torque controller with %zu joints",
        joints_.size());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WutVelmaEffortController::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::fill(
    reference_interfaces_.begin(),
    reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WutVelmaEffortController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (auto & cmd : command_interfaces_)
  {
    cmd.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

bool WutVelmaEffortController::on_set_chained_mode(bool chained_mode)
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
WutVelmaEffortController::update_reference_from_subscribers(
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
// WutVelmaEffortController::on_activate(const rclcpp_lifecycle::State &)
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
// WutVelmaEffortController::update(const rclcpp::Time &, const rclcpp::Duration &)
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
//         std::cout << "WutVelmaEffortController: could not calculate joint impedance" << std::endl;
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
WutVelmaEffortController::update_and_write_commands(
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
    VVector joint_position_cmd;
    VVector stiffness;
    VVector joint_velocity;
    VVector nullspace_torque_cmd;
    VMatrix mass_matrix;
    VVector out_joint_torque_command;



//   for (size_t i = 0; i < joints_.size(); ++i)
//   {
//     const double q_ref =
//       reference_interfaces_[ref_index(i, 0)];
//     const double dq_ref =
//       reference_interfaces_[ref_index(i, 1)];
//     const double ddq_ref =
//       reference_interfaces_[ref_index(i, 2)];

//     if (!std::isfinite(q_ref) ||
//         !std::isfinite(dq_ref) ||
//         !std::isfinite(ddq_ref))
//     {
//       // Nie pisz śmieci do effort, dopóki nie przyszła trajektoria.
//       command_interfaces_[i].set_value(0.0);
//       continue;
//     }

//     // state_interfaces_ są w kolejności:
//     // joint0/position
//     // joint0/velocity
//     // joint1/position
//     // joint1/velocity
//     // ...
//     const double q = state_interfaces_[2 * i + 0].get_value();
//     const double dq = state_interfaces_[2 * i + 1].get_value();

//     const double e = q_ref - q;
//     const double de = dq_ref - dq;

//     // Minimalny wariant:
//     // tau = feedforward acceleration + PD
//     //
//     // Docelowo tutaj podstawiasz:
//     // tau = M(q) * ddq_ref + C(q,dq) * dq_ref + g(q)
//     //       + Kp * (q_ref - q) + Kd * (dq_ref - dq)
//     //
//     // Dla pojedynczego niezależnego złącza można zacząć od:
//     const double tau = ddq_ref + kp_[i] * e + kd_[i] * de;

//     command_interfaces_[i].set_value(tau);
//   }

    bool no_cmd = false;

    for (size_t i = 0; i < joints_.size(); ++i) {
        auto q_opt = state_interfaces_[2*i + 0].get_optional();
        auto dq_opt = state_interfaces_[2*i + 1].get_optional();

        if (q_opt && dq_opt) {
            joint_position[i] = *q_opt;
            joint_velocity[i] = *dq_opt;
        }
        else {
            return controller_interface::return_type::ERROR;
        }

        // joint_position_cmd[i] = (*q_des)[i];

        const double q_ref =
        reference_interfaces_[ref_index(i, 0)];
        const double dq_ref =
        reference_interfaces_[ref_index(i, 1)];
        const double ddq_ref =
        reference_interfaces_[ref_index(i, 2)];

        if (!std::isfinite(q_ref) ||
            !std::isfinite(dq_ref) ||
            !std::isfinite(ddq_ref))
        {
            // Nie pisz śmieci do effort, dopóki nie przyszła trajektoria.
            // command_interfaces_[i].set_value(0.0);
            no_cmd = true;
            break;
        }

        joint_position_cmd[i] = q_ref;
    }

    if (!no_cmd) {
        velma_model_->setJointPosition(joint_position);
        velma_model_->calculateMassMatrix(mass_matrix);
        // mass_matrix.setConstant(1.0);

        // TODO: read 'stiffness'
        stiffness.setConstant(50.0);

        nullspace_torque_cmd.setZero();
        out_joint_torque_command.setZero();

        if (!jimp_.calculate(joint_position, joint_position_cmd, stiffness, joint_velocity, nullspace_torque_cmd, mass_matrix, out_joint_torque_command)) {
            std::cout << "WutVelmaEffortController: could not calculate joint impedance" << std::endl;
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
  wut_velma_effort_controller::WutVelmaEffortController,
  controller_interface::ChainableControllerInterface)
