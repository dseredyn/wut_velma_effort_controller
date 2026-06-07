#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"
#include "wut_velma_effort_controller/wut_velma_types.hpp"

namespace cartesian_trajectory_generator
{

class CartesianTrajectoryGenerator
  : public controller_interface::ChainableControllerInterface
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

protected:
    std::vector<hardware_interface::CommandInterface>
    on_export_reference_interfaces() override;

    bool on_set_chained_mode(bool chained_mode) override;

    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    using TrajectoryMsg = trajectory_msgs::msg::MultiDOFJointTrajectory;
    using TrajectoryMsgPtr = std::shared_ptr<const TrajectoryMsg>;

    enum RefIndex : std::size_t
    {
        PX = 0,
        PY,
        PZ,
        QX,
        QY,
        QZ,
        QW,
        // VX,
        // VY,
        // VZ,
        // WX,
        // WY,
        // WZ,
        // AX,
        // AY,
        // AZ,
        // ALPHAX,
        // ALPHAY,
        // ALPHAZ,
        REF_COUNT
    };

    struct Sample
    {
        std::array<geometry_msgs::msg::Transform, EEcount> transform;
        std::array<bool, EEcount> valid;
        // geometry_msgs::msg::Transform transform;
        // geometry_msgs::msg::Twist twist;
        // geometry_msgs::msg::Twist accel;
    };

    static double duration_to_seconds(const builtin_interfaces::msg::Duration & duration);

    static double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp);

    static geometry_msgs::msg::Transform interpolate_transform(
        const geometry_msgs::msg::Transform & a,
        const geometry_msgs::msg::Transform & b,
        double s);

    // static geometry_msgs::msg::Twist interpolate_twist(
    //     const geometry_msgs::msg::Twist & a,
    //     const geometry_msgs::msg::Twist & b,
    //     double s);

    // static geometry_msgs::msg::Twist zero_twist();

    // static geometry_msgs::msg::Twist constant_linear_velocity(
    //     const geometry_msgs::msg::Transform & a,
    //     const geometry_msgs::msg::Transform & b,
    //     double dt);

    bool validate_trajectory(const TrajectoryMsg & msg) const;

    bool sample_trajectory(
        const TrajectoryMsg & msg,
        double t_from_start,
        Sample & sample);

    void write_sample_to_reference_interfaces(const Sample & sample);

    void set_default_nan_reference();

    std::vector<std::string> make_cartesian_interface_names(
                            const std::array<std::string, EEcount> & cartesian_names) const;

    std::string downstream_controller_;

    std::vector<std::string> exported_reference_interface_names_;
    std::vector<std::string> downstream_command_interface_names_;

    rclcpp::Subscription<TrajectoryMsg>::SharedPtr trajectory_sub_;
    realtime_tools::RealtimeBuffer<TrajectoryMsgPtr> trajectory_buffer_;

    TrajectoryMsgPtr active_trajectory_;
    rclcpp::Time active_trajectory_start_time_{0, 0, RCL_ROS_TIME};

    bool chained_mode_ = false;
    bool has_valid_reference_ = false;

    std::array<std::string, EEcount> ee_names_;

    int dbg_prev_traj_pt_, dbg_current_traj_pt_;
};

}  // namespace cartesian_trajectory_generator
