// TODO: license

#include "wut_velma_effort_controller/wut_velma_effort_controller.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "wut_velma_effort_controller/wut_velma_model.hpp"

namespace wut_velma_effort_controller
{

VisualizationSnapshot::VisualizationSnapshot()
: collisions_count(0)
, spheres_count(0)
, capsules_count(0)
{}

inline std::string get_urdf_from_topic_detached(
  const std::string & topic_name = "/robot_description",
  std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
{
    // Create a temporary node with its own executor.
    // Subscribe to /robot_description topic to fetch one message.
    // Spin until message is received.
    // Stop and remove the node.

    auto tmp_node = std::make_shared<rclcpp::Node>("urdf_fetcher");

    auto prom = std::make_shared<std::promise<std::string>>();
    auto fut  = prom->get_future();

    // Synchronization for set_value in the callback.
    auto done = std::make_shared<std::atomic_bool>(false);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    std::function<void(std_msgs::msg::String::ConstSharedPtr)> cb =
        [prom, done](std_msgs::msg::String::ConstSharedPtr msg)
        {
        if (!msg || msg->data.empty()) return;

        bool expected = false;
        if (!done->compare_exchange_strong(expected, true)) return; // already done

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
    auto_declare<double>("collision_distance", -1.0);
    auto_declare<std::vector<std::string>>("collision_model_links", std::vector<std::string>{});
    auto_declare<int>("collision_model_groups", 0);
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
    std::string srdf_xml;

    // Read the URDF
    try {
        urdf_xml = get_urdf_from_topic_detached("/robot_description", std::chrono::milliseconds(3000));
        RCLCPP_INFO(get_node()->get_logger(), "Got robot_description (%zu bytes).", urdf_xml.size());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    // else

    // Read the SRDF
    try {
        srdf_xml = get_urdf_from_topic_detached("/robot_description_semantic", std::chrono::milliseconds(20000));
        RCLCPP_INFO(get_node()->get_logger(), "Got robot_description_semantic (%zu bytes).", srdf_xml.size());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description_semantic: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    // else

    joints_ = node->get_parameter("joints").as_string_array();

    if (joints_.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' is empty.");
        return controller_interface::CallbackReturn::ERROR;
    }
    // else

    if (Vjoints != joints_.size()) {
        RCLCPP_ERROR(node->get_logger(), "Wrong number of joints in controller, expected 15");
        return controller_interface::CallbackReturn::ERROR;
    }
    // else

    // Print all controlled joints
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        RCLCPP_INFO_STREAM(node->get_logger(), "joint " << i <<": \"" << joints_[i]);
        // std::cout << "WutVelmaEffortController joint " << i <<": \"" << joints_[i] << "\""
                                                                                    // << std::endl;
    }

    // Read collision model-specific parameters
    // Read collision_distance
    rclcpp::Parameter param_collision_distance;
    if (!node->get_parameter("collision_distance", param_collision_distance)) {
        RCLCPP_ERROR(node->get_logger(), "Parameter \"collision_distance\" is not set");
        return controller_interface::CallbackReturn::ERROR;
    }
    // else
    collision_distance_ = param_collision_distance.as_double();

    if (collision_distance_ < 0) {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Parameter \"collision_distance\" has wrong value: " << collision_distance_);
        return controller_interface::CallbackReturn::ERROR;
    }

    // Read collision model groups
    rclcpp::Parameter param_collision_model_groups;
    if (!node->get_parameter("collision_model_groups", param_collision_model_groups)) {
        RCLCPP_ERROR(node->get_logger(), "Parameter \"collision_model_groups\" is not set");
        return controller_interface::CallbackReturn::ERROR;
    }
    // else
    int collision_model_groups = param_collision_model_groups.as_int();
    std::vector<std::vector<std::string>> groups;
    for (int i = 0; i < collision_model_groups; ++i) {
        auto param_name = "collision_model_group" + std::to_string(i);
        std::vector<std::string> gr = 
                        node->declare_parameter<std::vector<std::string>>(
                            param_name,
                            std::vector<std::string>());
        groups.push_back(gr);
    }

    std::vector<std::pair<std::string, std::string>> collision_pairs;
    for (size_t i = 0; i < groups.size()-1; ++i) {
        const auto& gr0 = groups[i];
        for (size_t j = i+1; j < groups.size(); ++j) {
            const auto& gr1 = groups[j];
            for (size_t k = 0; k < gr0.size(); ++k) {
                for (size_t l = 0; l < gr1.size(); ++l) {
                    collision_pairs.push_back( std::make_pair(gr0[k], gr1[l]) );
                }
            }
        }
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "Collision pairs: " << collision_pairs.size());

    // Read collision model capsules and spheres
    std::vector<std::string> collision_link_names = node->get_parameter("collision_model_links").as_string_array();

    std::vector<CollisionGeomSharedPtr> collision_link_geoms;
    for (size_t i = 0; i < collision_link_names.size(); ++i) {
        auto param_name = "collision_model_col" + std::to_string(i);
        std::vector<double> gp = 
                        node->declare_parameter<std::vector<double>>(
                            param_name,
                            std::vector<double>());
        CollisionGeomSharedPtr geom;
        if (gp.size() != 4 && gp.size() != 8) {
            RCLCPP_ERROR_STREAM(node->get_logger(), "Wrong size for parameter \"" << param_name << "\": " << gp.size());
            return controller_interface::CallbackReturn::ERROR;
        }
        if (gp.size() == 4) {
            RCLCPP_INFO_STREAM(node->get_logger(), "Collision geometry for link \""
                        << collision_link_names[i] << "\": sphere r=" << gp[0]);
            geom.reset( new CollisionSphere(gp[0], Eigen::Vector3d(gp[1], gp[2], gp[3])) );
        }
        
        if (gp.size() == 8) {
            RCLCPP_INFO_STREAM(node->get_logger(), "Collision geometry for link \""
                        << collision_link_names[i] << "\": capsule l= " << gp[0] << ", r=" << gp[1]);
            geom.reset( new CollisionCapsule(gp[0], gp[1], Eigen::Vector3d(gp[2], gp[3], gp[4]),
                                                            Eigen::Vector3d(gp[5], gp[6], gp[7])) );
        }
        collision_link_geoms.push_back( geom );
    }

    RCLCPP_INFO_STREAM(node->get_logger(), "Using collision_distance: " << collision_distance_);

    RCLCPP_INFO(node->get_logger(), "Creating WutVelmaModel...");

    // TODO: fk links
    std::vector<std::string> links;

    links = collision_link_names;   // TODO: Is this enough?

    velma_model_.emplace(urdf_xml, joints_, links, srdf_xml, collision_distance_, collision_link_names, collision_link_geoms, collision_pairs);

    col_data_buf_.resize(2);
    col_data_buf_idx_ = 0;
    for (size_t i = 0; i < col_data_buf_.size(); ++i) {
        col_data_buf_[i].resize(velma_model_->getCollisionPairsCount());
    }

    RCLCPP_INFO(node->get_logger(), "Created WutVelmaModel.");

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
        // low-level command (outputs): command interface hardware
        command_interface_names_.push_back(
        joint + "/" + hardware_interface::HW_IF_EFFORT);

        // low-level state (inputs): state interfaces hardware
        state_interface_names_.push_back(
        joint + "/" + hardware_interface::HW_IF_POSITION);
        state_interface_names_.push_back(
        joint + "/" + hardware_interface::HW_IF_VELOCITY);

        // high-level commands (inputs)
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


    marker_pub_ =
        get_node()->create_publisher<
            visualization_msgs::msg::MarkerArray>(
            "~/collision_markers",
            rclcpp::QoS(1).best_effort());

    using namespace std::chrono_literals;

    marker_timer_ =
        get_node()->create_wall_timer(
            50ms,  // 20 Hz is plenty for RViz
            [this]()
            {
            publish_markers();
            });

    marker_timer_->cancel();

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WutVelmaEffortController::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::fill(
    reference_interfaces_.begin(),
    reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

  marker_pub_->on_activate();
  marker_timer_->reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WutVelmaEffortController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (auto & cmd : command_interfaces_)
  {
    cmd.set_value(0.0);
  }

  marker_timer_->cancel();
  marker_pub_->on_deactivate();

  return controller_interface::CallbackReturn::SUCCESS;
}

bool WutVelmaEffortController::on_set_chained_mode(bool chained_mode)
{
    // Only chained mode is supported.
    chained_mode_ = chained_mode;
    return true;
}

controller_interface::return_type
WutVelmaEffortController::update_reference_from_subscribers(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
    // This is not used in chained mode.
    // Only chained mode is supported.
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
    VVector joint_position_cmd_new;
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

    bool received_command = true;

    // Read the commands from high-level controller.
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
            received_command = false;
            break;
        }

        joint_position_cmd_new[i] = q_ref;
    }

    // TODO: if no high-level command is received, report an error and use fallback controller (return ERROR).
    // return controller_interface::return_type::ERROR;

    // TODO: stop trajectory interpolation in case of risk of collision.
    // TODO: resume trajectory interpolation in case of collision and leaving trajectory.

    if (received_command) {

        // Calculate self-collisions for the commanded configuration
        velma_model_->setJointPosition(joint_position_cmd_new);
        velma_model_->calculateFk();

        auto& current_col_data_buf = col_data_buf_[col_data_buf_idx_];

        velma_model_->calculateSelfCollisions(current_col_data_buf);

        // TODO: check for a self-collision condition
        bool is_in_self_collision = false;
        for (size_t i = 0; i < current_col_data_buf.size(); ++i) {
            if (current_col_data_buf[i].distance < collision_distance_) {
                is_in_self_collision = true;
                RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                            1000, "is_in_self_collision: " << current_col_data_buf[i].link0_idx
                            << " - " << current_col_data_buf[i].link1_idx << ", dist: "
                            << current_col_data_buf[i].distance);
                break;
            }
        }

        if (is_in_self_collision) {
            // Keep the old commanded configuration
        }
        else {
            // The new commanded configuration is ok
            joint_position_cmd_ = joint_position_cmd_new;
            col_data_buf_idx_ = (col_data_buf_idx_+1) % col_data_buf_.size();
        }

        if (!processDebugVisualization(current_col_data_buf)) {
            return controller_interface::return_type::ERROR;
        }
    
        // Use the command to calculate the output low-level command.
        // This time, use the actual, current configuration of the robot.
        velma_model_->setJointPosition(joint_position);
        velma_model_->calculateFk();
        velma_model_->calculateMassMatrix(mass_matrix);
        // mass_matrix.setConstant(1.0);

        // TODO: read 'stiffness'
        stiffness.setConstant(50.0);

        nullspace_torque_cmd.setZero();
        out_joint_torque_command.setZero();

        if (!jimp_.calculate(joint_position, joint_position_cmd_, stiffness, joint_velocity, nullspace_torque_cmd, mass_matrix, out_joint_torque_command)) {
            std::cout << "WutVelmaEffortController: could not calculate joint impedance" << std::endl;
            return controller_interface::return_type::ERROR;
        }
    }
    else {
        // No high-level command is received, set the commanded output to 0.
        for (size_t i = 0; i < joints_.size(); ++i) {
            out_joint_torque_command[i] = 0.0;
        }
    }

    // Send the low-level command.
    // state_interfaces_ holds [q0, dq0, q1, dq1, ...] as in the state_interface_configuration
    for (size_t i = 0; i < joints_.size(); ++i) {
        if (!command_interfaces_[i].set_value(out_joint_torque_command[i])) {
            return controller_interface::return_type::ERROR;
        }
    }
    return controller_interface::return_type::OK;
}

geometry_msgs::msg::Point cPoint(const Eigen::Vector3d& p) {
    geometry_msgs::msg::Point result;
    result.x = p.x();
    result.y = p.y();
    result.z = p.z();
    return result;
}


bool WutVelmaEffortController::processDebugVisualization(
                                                    const std::vector<CollisionData>& coll_data) {
    // Visualization for debug
    VisualizationSnapshot snapshot;
    snapshot.collisions_count = 0;
    for (size_t i = 0; i < coll_data.size(); ++i) {
        if (coll_data[i].distance < collision_distance_) {
            snapshot.collisions[snapshot.collisions_count++] = coll_data[i];
            if (snapshot.collisions_count >= snapshot.collisions.size()) {
                break;
            }
        }
    }

    snapshot.spheres_count = 0;
    snapshot.capsules_count = 0;
    auto col_geoms = velma_model_->getCollisionGeoms();
    for (size_t i = 0; i < col_geoms.size(); ++i) {
        if (col_geoms[i]->getType() == CollisionGeom::SPHERE) {
            if (snapshot.spheres_count >= snapshot.spheres.size()) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Too many collision spheres: " << snapshot.spheres_count);
                return false;
            }
            snapshot.spheres[snapshot.spheres_count++] = *std::static_pointer_cast<CollisionSphere>(col_geoms[i]);
        }
        else if (col_geoms[i]->getType() == CollisionGeom::CAPSULE) {
            if (snapshot.capsules_count >= snapshot.capsules.size()) {
                RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Too many collision capsules: " << snapshot.capsules_count);
                return false;
            }
            snapshot.capsules[snapshot.capsules_count++] = *std::static_pointer_cast<CollisionCapsule>(col_geoms[i]);
        }
    }

    // Never wait for visualization.
    // If the non-RT thread happens to have the object locked,
    // simply drop this visualization update.
    (void)visualization_box_.try_set(snapshot);
    return true;
}

void WutVelmaEffortController::publish_markers()
{
    const auto snapshot = visualization_box_.try_get();

    if (!snapshot)
        return;

    visualization_msgs::msg::MarkerArray msg;

    for (size_t i = 0; i < snapshot->collisions.size(); ++i) {
        visualization_msgs::msg::Marker m;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.header.frame_id = "torso_base";
        m.ns = "distances";
        m.id = i;
        if (i < snapshot->collisions_count) {
            m.action = visualization_msgs::msg::Marker::ADD;
            m.points.push_back(cPoint(snapshot->collisions[i].p0));
            m.points.push_back(cPoint(snapshot->collisions[i].p1));
            m.scale.x = 0.01;
            m.scale.y = 0.01;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 0.0;
            m.color.a = 1.0;
        }
        else {
            m.action = visualization_msgs::msg::Marker::DELETE;
        }
        msg.markers.push_back(m);
    }

    for (size_t i = 0; i < snapshot->spheres_count; ++i) {
        visualization_msgs::msg::Marker m;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.header.frame_id = "torso_base";
        m.ns = "spheres";
        m.id = i;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        m.color.a = 0.5;
        m.scale.x = m.scale.y = m.scale.z = snapshot->spheres[i].radius * 2;
        m.pose.position.x = snapshot->spheres[i].p_g.x();
        m.pose.position.y = snapshot->spheres[i].p_g.y();
        m.pose.position.z = snapshot->spheres[i].p_g.z();
        msg.markers.push_back(m);
    }

    for (size_t i = 0; i < snapshot->capsules_count; ++i) {
        visualization_msgs::msg::Marker m;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.header.frame_id = "torso_base";
        m.ns = "capsules";
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = m.scale.y = m.scale.z = snapshot->capsules[i].radius * 2;
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        m.color.a = 0.5;
        m.id = i*3;
        m.pose.position.x = snapshot->capsules[i].p0_g.x();
        m.pose.position.y = snapshot->capsules[i].p0_g.y();
        m.pose.position.z = snapshot->capsules[i].p0_g.z();
        msg.markers.push_back(m);
        m.id = i*3+1;
        m.pose.position.x = snapshot->capsules[i].p1_g.x();
        m.pose.position.y = snapshot->capsules[i].p1_g.y();
        m.pose.position.z = snapshot->capsules[i].p1_g.z();
        msg.markers.push_back(m);

        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.id = i*3+2;
        m.scale.z = snapshot->capsules[i].length;
        auto d = snapshot->capsules[i].p1_g - snapshot->capsules[i].p0_g;
        const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
                                                    Eigen::Vector3d::UnitZ(), d);
        auto center = (snapshot->capsules[i].p0_g + snapshot->capsules[i].p1_g) / 2;
        m.pose.position.x = center.x();
        m.pose.position.y = center.y();
        m.pose.position.z = center.z();
        m.pose.orientation.x = q.x();
        m.pose.orientation.y = q.y();
        m.pose.orientation.z = q.z();
        m.pose.orientation.w = q.w();
        msg.markers.push_back(m);
    }

    marker_pub_->publish(msg);
}

}  // namespace wut_velma_effort_controller

PLUGINLIB_EXPORT_CLASS(
  wut_velma_effort_controller::WutVelmaEffortController,
  controller_interface::ChainableControllerInterface)
