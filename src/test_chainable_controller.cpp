// TODO: license

#include "wut_velma_effort_controller/test_chainable_controller.hpp"

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


controller_interface::CallbackReturn TestChainableController::on_init()
{
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::string>("downstream_command_controller", std::string());
    auto_declare<std::string>("downstream_state_controller", std::string());
    auto_declare<bool>("require_state_interfaces", false);
    auto_declare<bool>("require_command_interfaces", false);
    return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::InterfaceConfiguration
TestChainableController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = command_interface_names_;
  return config;
}

controller_interface::InterfaceConfiguration
TestChainableController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = state_interface_names_;
  return config;
}

std::vector<hardware_interface::CommandInterface>
TestChainableController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(exported_reference_interface_names_.size());

  for (size_t i = 0; i < exported_reference_interface_names_.size(); ++i)
  {
    interfaces.emplace_back(
      get_node()->get_name(),
      exported_reference_interface_names_[i],
      &reference_interfaces_[i]);
  }

  return interfaces;
}

std::vector<hardware_interface::StateInterface>
TestChainableController::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(exported_state_interface_names_.size());

  for (size_t i = 0; i < exported_state_interface_names_.size(); ++i)
  {
    interfaces.emplace_back(
      get_node()->get_name(),
      exported_state_interface_names_[i],
      &state_interfaces_values_[i]);
  }

  return interfaces;
}

controller_interface::CallbackReturn
TestChainableController::on_configure(const rclcpp_lifecycle::State &)
{
    auto node = get_node();

#ifdef NDEBUG
  std::cout << "Release (NDEBUG defined)\n";
#else
  std::cout << "Debug (NDEBUG not defined)\n";
#endif

    joints_ = node->get_parameter("joints").as_string_array();

    if (joints_.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' is empty.");
        return controller_interface::CallbackReturn::ERROR;
    }

    downstream_command_controller_ = node->get_parameter("downstream_command_controller").as_string();
    downstream_state_controller_ = node->get_parameter("downstream_state_controller").as_string();
    require_command_interfaces_ = node->get_parameter("require_command_interfaces").as_bool();
    require_state_interfaces_ = node->get_parameter("require_state_interfaces").as_bool();
    // export_reference_interfaces_ = true; //node->get_parameter("export_reference_interfaces").as_bool();
    // export_state_interfaces_ = true; // node->get_parameter("export_state_interfaces").as_bool();

    // Print all controlled joints
    for (std::size_t i = 0; i < joints_.size(); ++i) {
        std::cout << "TestChainableController joint " << i <<": \"" << joints_[i] << "\""
                                                                                    << std::endl;
    }

    command_interface_names_.clear();
    state_interface_names_.clear();
    exported_reference_interface_names_.clear();
    exported_state_interface_names_.clear();

    for (const auto & joint : joints_)
    {
        // Command outputs
        if (require_command_interfaces_) {
            if (downstream_command_controller_.empty()) {
                command_interface_names_.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
            }
            else {
                command_interface_names_.push_back(downstream_command_controller_ + "/" + joint + "/" + hardware_interface::HW_IF_POSITION);
            }
        }

        // State inputs
        if (require_state_interfaces_) {
            if (downstream_state_controller_.empty()) {
                state_interface_names_.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
            }
            else {
                state_interface_names_.push_back(downstream_state_controller_ + "/" + joint + "/" + hardware_interface::HW_IF_POSITION);
            }
        }

        // Command inputs
        // if (export_reference_interfaces_) {
        exported_reference_interface_names_.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        // }

        // State outputs
        // if (export_state_interfaces_) {
        exported_state_interface_names_.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        // }
    }

    reference_interfaces_.assign(
        exported_reference_interface_names_.size(),
        std::numeric_limits<double>::quiet_NaN());

    state_interfaces_values_.assign(
        exported_state_interface_names_.size(),
        std::numeric_limits<double>::quiet_NaN());

    RCLCPP_INFO(
        get_node()->get_logger(),
        "Configured chainable computed torque controller with %zu joints",
        joints_.size());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TestChainableController::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::fill(
    reference_interfaces_.begin(),
    reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

  // TODO: is this necessary?
  std::fill(
    state_interfaces_values_.begin(),
    state_interfaces_values_.end(),
    std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TestChainableController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (auto & cmd : command_interfaces_)
  {
    cmd.set_value(0.0);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

bool TestChainableController::on_set_chained_mode(bool chained_mode)
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
TestChainableController::update_reference_from_subscribers(
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

controller_interface::return_type
TestChainableController::update_and_write_commands(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
    for (size_t i = 0; i < state_interfaces_.size(); ++i) {
        auto q_opt = state_interfaces_[i].get_optional();

        if (q_opt) {
            double q = *q_opt;
        }
        else {
            return controller_interface::return_type::ERROR;
        }
    }

    for (size_t i = 0; i < reference_interfaces_.size(); ++i) {
        const double q_ref =
        reference_interfaces_[i];
        if (std::isfinite(q_ref)) {
            // ok
        }
    }

    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
        if (!command_interfaces_[i].set_value(0.0)) {
            return controller_interface::return_type::ERROR;
        }
    }
    for (size_t i = 0; i < state_interfaces_values_.size(); ++i) {
        state_interfaces_values_[i] = 0.0;
    }

    return controller_interface::return_type::OK;
}


}  // namespace wut_velma_effort_controller

PLUGINLIB_EXPORT_CLASS(
  wut_velma_effort_controller::TestChainableController,
  controller_interface::ChainableControllerInterface)
