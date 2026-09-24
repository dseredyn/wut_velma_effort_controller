// TODO: license

#pragma once

#include <array>
#include <optional>

#include "controller_interface/chainable_controller_interface.hpp"
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_buffer.hpp>

// For debug marker publishing
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "wut_velma_effort_controller/joint_impedance.hpp"
#include "wut_velma_effort_controller/wut_velma_model.hpp"

namespace wut_velma_effort_controller
{

class VisualizationSnapshot {
public:
    VisualizationSnapshot();
    std::array<CollisionData, 100> collisions;
    size_t collisions_count;

    std::array<CollisionSphere, 10> spheres;
    size_t spheres_count;
    std::array<CollisionCapsule, 35> capsules;
    size_t capsules_count;
};

class WutVelmaEffortController : public controller_interface::ChainableControllerInterface
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

    bool processDebugVisualization(const std::vector<CollisionData>& coll_data);

    // controller_interface::CallbackReturn on_init() override;

    // controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    // controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    // controller_interface::CallbackReturn on_configure(
    //     const rclcpp_lifecycle::State & previous_state) override;

    // controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

    // controller_interface::return_type update_and_write_commands(
    //     const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    size_t ref_index(size_t joint_index, size_t ref_type_index) const
    {
        // ref_type_index:
        // 0 -> position
        // 1 -> velocity
        // 2 -> acceleration
        return joint_index * 3 + ref_type_index;
    }

    // For debug marker publishing
    realtime_tools::RealtimeThreadSafeBox<VisualizationSnapshot> visualization_box_{VisualizationSnapshot{}};
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr marker_timer_;
    void publish_markers();

    std::vector<std::string> joints_;

    // realtime_tools::RealtimeBuffer<std::vector<double>> q_des_buf_;
    // rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;

    JointImpedance jimp_;
    std::optional<WutVelmaModel> velma_model_;

    VVector joint_position_cmd_;

    std::vector<std::string> command_interface_names_;
    std::vector<std::string> state_interface_names_;
    std::vector<std::string> reference_interface_names_;

    bool chained_mode_ = false;

    // For self-collision detection
    double collision_distance_;
    std::vector<std::vector<CollisionData>> col_data_buf_;
    size_t col_data_buf_idx_;
};

}  // namespace wut_velma_effort_controller
