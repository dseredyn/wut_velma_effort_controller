// TODO: license

#pragma once

#include <optional>

#include "controller_interface/chainable_controller_interface.hpp"
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "wut_velma_effort_controller/cartesian_impedance.hpp"
#include "wut_velma_effort_controller/wut_velma_model.hpp"

namespace wut_velma_effort_controller
{

class TfDebug;
typedef std::unique_ptr<TfDebug> TfDebugUniquePtr;

class CartTorqueController : public controller_interface::ChainableControllerInterface
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

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    std::vector<std::string> make_cartesian_interface_names(
                                const std::array<std::string, EEcount> & cartesian_names) const;

private:
    // size_t ref_index(size_t joint_index, size_t ref_type_index) const
    // {
    //     // ref_type_index:
    //     // 0 -> position
    //     // 1 -> velocity
    //     // 2 -> acceleration
    //     return joint_index * 3 + ref_type_index;
    // }
    
    std::vector<std::string> joints_;

    realtime_tools::RealtimeBuffer<std::vector<double>> q_des_buf_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;

    CartesianImpedance cimp_;
    std::optional<WutVelmaModel> velma_model_;

    std::vector<std::string> command_interface_names_;
    std::vector<std::string> state_interface_names_;
    std::vector<std::string> reference_interface_names_;

    bool chained_mode_ = false;

    // for debug
    std::array<Eigen::Affine3d, EEcount> cart_position_cmd_;

    // tf debug
    TfDebugUniquePtr tf_debug_;
    int publish_tf_every_n_updates_ = 100;
    int tf_publish_counter_ = 0;
};

}  // namespace wut_velma_effort_controller
