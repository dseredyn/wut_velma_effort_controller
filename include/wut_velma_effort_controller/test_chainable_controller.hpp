// TODO: license

#pragma once

#include <optional>

#include "controller_interface/chainable_controller_interface.hpp"
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_buffer.hpp>

namespace wut_velma_effort_controller
{

class TestChainableController : public controller_interface::ChainableControllerInterface
{
public:
    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    bool on_set_chained_mode(bool chained_mode) override;

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

protected:
    std::vector<hardware_interface::CommandInterface>
    on_export_reference_interfaces() override;

    std::vector<hardware_interface::StateInterface>
    on_export_state_interfaces() override;

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    size_t ref_index(size_t joint_index, size_t ref_type_index) const
    {
        // ref_type_index:
        // 0 -> position
        // 1 -> velocity
        // 2 -> acceleration
        return joint_index * 3 + ref_type_index;
    }
    
    std::vector<std::string> joints_;
    std::string downstream_command_controller_;
    std::string downstream_state_controller_;
    bool require_command_interfaces_;
    bool require_state_interfaces_;
    // bool export_reference_interfaces_;
    // bool export_state_interfaces_;

    realtime_tools::RealtimeBuffer<std::vector<double>> q_des_buf_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;

    std::vector<std::string> command_interface_names_;
    std::vector<std::string> state_interface_names_;
    // std::vector<std::string> exported_reference_interfaces_names_;
    // std::vector<std::string> exported_state_interfaces_names_;

    bool chained_mode_ = false;
};

}  // namespace wut_velma_effort_controller
