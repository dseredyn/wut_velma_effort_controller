// TODO: license

#include "wut_velma_effort_controller/cartesian_trajectory_generator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "wut_velma_effort_controller/wut_velma_types.hpp"

namespace cartesian_trajectory_generator
{
namespace
{
constexpr double kEps = 1e-12;

struct Quaternion
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

Quaternion normalize(Quaternion q)
{
    const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (n < kEps)
    {
        return Quaternion{};
    }
    q.x /= n;
    q.y /= n;
    q.z /= n;
    q.w /= n;
    return q;
}

Quaternion slerp(Quaternion q0, Quaternion q1, double t)
{
    q0 = normalize(q0);
    q1 = normalize(q1);

    double dot = q0.x * q1.x + q0.y * q1.y + q0.z * q1.z + q0.w * q1.w;

    if (dot < 0.0)
    {
        q1.x = -q1.x;
        q1.y = -q1.y;
        q1.z = -q1.z;
        q1.w = -q1.w;
        dot = -dot;
    }

    if (dot > 0.9995)
    {
        Quaternion out;
        out.x = q0.x + t * (q1.x - q0.x);
        out.y = q0.y + t * (q1.y - q0.y);
        out.z = q0.z + t * (q1.z - q0.z);
        out.w = q0.w + t * (q1.w - q0.w);
        return normalize(out);
    }

    dot = std::clamp(dot, -1.0, 1.0);
    const double theta_0 = std::acos(dot);
    const double theta = theta_0 * t;
    const double sin_theta = std::sin(theta);
    const double sin_theta_0 = std::sin(theta_0);

    const double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
    const double s1 = sin_theta / sin_theta_0;

    Quaternion out;
    out.x = s0 * q0.x + s1 * q1.x;
    out.y = s0 * q0.y + s1 * q1.y;
    out.z = s0 * q0.z + s1 * q1.z;
    out.w = s0 * q0.w + s1 * q1.w;
    return normalize(out);
}

bool finite_transform(const geometry_msgs::msg::Transform & t)
{
    return std::isfinite(t.translation.x) &&
            std::isfinite(t.translation.y) &&
            std::isfinite(t.translation.z) &&
            std::isfinite(t.rotation.x) &&
            std::isfinite(t.rotation.y) &&
            std::isfinite(t.rotation.z) &&
            std::isfinite(t.rotation.w);
}

// bool finite_twist(const geometry_msgs::msg::Twist & t)
// {
//   return std::isfinite(t.linear.x) &&
//          std::isfinite(t.linear.y) &&
//          std::isfinite(t.linear.z) &&
//          std::isfinite(t.angular.x) &&
//          std::isfinite(t.angular.y) &&
//          std::isfinite(t.angular.z);
// }
}  // namespace

controller_interface::CallbackReturn CartesianTrajectoryGenerator::on_init()
{
    ee_names_[EEidxL] = "arm_l";
    ee_names_[EEidxR] = "arm_r";

    try
    {
        auto_declare<std::string>("downstream_controller", "computed_cart_torque_controller");
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianTrajectoryGenerator::on_configure(
  const rclcpp_lifecycle::State &)
{
    downstream_controller_ = get_node()->get_parameter("downstream_controller").as_string();

    if (downstream_controller_.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'downstream_controller' is empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    exported_reference_interface_names_ = make_cartesian_interface_names(ee_names_);

    downstream_command_interface_names_.clear();
    downstream_command_interface_names_.reserve(exported_reference_interface_names_.size());
    for (const auto & name : exported_reference_interface_names_)
    {
        downstream_command_interface_names_.push_back(downstream_controller_ + "/" + name);
    }

    reference_interfaces_.assign(REF_COUNT*EEcount, std::numeric_limits<double>::quiet_NaN());

    TrajectoryMsgPtr null_trajectory;
    trajectory_buffer_.writeFromNonRT(null_trajectory);

    trajectory_sub_ = get_node()->create_subscription<TrajectoryMsg>(
        "~/trajectory",
        rclcpp::SystemDefaultsQoS(),
        [this](TrajectoryMsg::ConstSharedPtr msg)
        {
        if (chained_mode_)
        {
            return;
        }

        if (!validate_trajectory(*msg))
        {
            RCLCPP_WARN(get_node()->get_logger(), "Rejected invalid Cartesian trajectory");
            return;
        }

        trajectory_buffer_.writeFromNonRT(msg);
        });

    RCLCPP_INFO(
        get_node()->get_logger(),
        "Configured CartesianTrajectoryGenerator: output -> %s/*",
        downstream_controller_.c_str());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianTrajectoryGenerator::on_activate(
    const rclcpp_lifecycle::State &)
{
    set_default_nan_reference();
    has_valid_reference_ = false;
    active_trajectory_.reset();

    TrajectoryMsgPtr null_trajectory;
    trajectory_buffer_.writeFromNonRT(null_trajectory);

    dbg_prev_traj_pt_ = dbg_current_traj_pt_ = -3;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianTrajectoryGenerator::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    set_default_nan_reference();
    has_valid_reference_ = false;
    active_trajectory_.reset();

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianTrajectoryGenerator::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = downstream_command_interface_names_;
    return config;
}

controller_interface::InterfaceConfiguration
CartesianTrajectoryGenerator::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
}

std::vector<hardware_interface::CommandInterface>
CartesianTrajectoryGenerator::on_export_reference_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(exported_reference_interface_names_.size());

    for (std::size_t i = 0; i < exported_reference_interface_names_.size(); ++i)
    {
        interfaces.emplace_back(
        get_node()->get_name(),
        exported_reference_interface_names_[i],
        &reference_interfaces_[i]);
    }

    return interfaces;
}

bool CartesianTrajectoryGenerator::on_set_chained_mode(bool chained_mode)
{
    chained_mode_ = chained_mode;

    if (chained_mode_)
    {
        RCLCPP_INFO(get_node()->get_logger(), "Entering chained mode; ~/trajectory input is ignored");
    }
    else
    {
        RCLCPP_INFO(get_node()->get_logger(), "Leaving chained mode; ~/trajectory input is active");
    }

    return true;
}

controller_interface::return_type
CartesianTrajectoryGenerator::update_reference_from_subscribers(
    const rclcpp::Time & time,
    const rclcpp::Duration &)
    {
    const auto trajectory_from_buffer = trajectory_buffer_.readFromRT();
    if (trajectory_from_buffer == nullptr || *trajectory_from_buffer == nullptr)
    {
        return controller_interface::return_type::OK;
    }

    if (*trajectory_from_buffer != active_trajectory_)
    {
        active_trajectory_ = *trajectory_from_buffer;

        if (stamp_to_seconds(active_trajectory_->header.stamp) > 0.0)
        {
            active_trajectory_start_time_ = rclcpp::Time(active_trajectory_->header.stamp);
        }
        else
        {
            active_trajectory_start_time_ = time;
        }
    }

    const double t_from_start = (time - active_trajectory_start_time_).seconds();

    dbg_prev_traj_pt_ = dbg_current_traj_pt_;
    Sample sample;
    if (!sample_trajectory(*active_trajectory_, t_from_start, sample))
    {
        if (dbg_prev_traj_pt_ != dbg_current_traj_pt_) {
            std::cout << "new trajectory point (could not sample): " << dbg_prev_traj_pt_ << " -> " << dbg_current_traj_pt_ << std::endl;
        }
        return controller_interface::return_type::OK;
    }

    if (dbg_prev_traj_pt_ != dbg_current_traj_pt_) {
            std::cout << "new trajectory point: " << dbg_prev_traj_pt_ << " -> " << dbg_current_traj_pt_ << std::endl;
    }

    write_sample_to_reference_interfaces(sample);
    has_valid_reference_ = true;

    return controller_interface::return_type::OK;
}

controller_interface::return_type
CartesianTrajectoryGenerator::update_and_write_commands(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
    bool finite_reference = true;
    for (const auto value : reference_interfaces_)
    {
        finite_reference = finite_reference && std::isfinite(value);
    }

    if (!finite_reference)
    {
        return controller_interface::return_type::OK;
    }

    if (!has_valid_reference_ && !chained_mode_)
    {
        return controller_interface::return_type::OK;
    }

    const std::size_t n = std::min(command_interfaces_.size(), reference_interfaces_.size());
    for (std::size_t i = 0; i < n; ++i)
    {
        command_interfaces_[i].set_value(reference_interfaces_[i]);
    }

    return controller_interface::return_type::OK;
}

std::vector<std::string> CartesianTrajectoryGenerator::make_cartesian_interface_names(
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

void CartesianTrajectoryGenerator::set_default_nan_reference()
{
    std::fill(
        reference_interfaces_.begin(),
        reference_interfaces_.end(),
        std::numeric_limits<double>::quiet_NaN());
}

double CartesianTrajectoryGenerator::duration_to_seconds(
  const builtin_interfaces::msg::Duration & duration)
{
    return static_cast<double>(duration.sec) + 1e-9 * static_cast<double>(duration.nanosec);
}

double CartesianTrajectoryGenerator::stamp_to_seconds(
  const builtin_interfaces::msg::Time & stamp)
{
    return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

// geometry_msgs::msg::Twist CartesianTrajectoryGenerator::zero_twist()
// {
//     geometry_msgs::msg::Twist out;
//     out.linear.x = 0.0;
//     out.linear.y = 0.0;
//     out.linear.z = 0.0;
//     out.angular.x = 0.0;
//     out.angular.y = 0.0;
//     out.angular.z = 0.0;
//     return out;
// }

// geometry_msgs::msg::Twist CartesianTrajectoryGenerator::constant_linear_velocity(
//   const geometry_msgs::msg::Transform & a,
//   const geometry_msgs::msg::Transform & b,
//   double dt)
// {
//     geometry_msgs::msg::Twist out = zero_twist();

//     if (dt > kEps)
//     {
//         out.linear.x = (b.translation.x - a.translation.x) / dt;
//         out.linear.y = (b.translation.y - a.translation.y) / dt;
//         out.linear.z = (b.translation.z - a.translation.z) / dt;
//     }

//     return out;
// }

geometry_msgs::msg::Transform CartesianTrajectoryGenerator::interpolate_transform(
  const geometry_msgs::msg::Transform & a,
  const geometry_msgs::msg::Transform & b,
  double s)
{
    geometry_msgs::msg::Transform out;

    out.translation.x = a.translation.x + s * (b.translation.x - a.translation.x);
    out.translation.y = a.translation.y + s * (b.translation.y - a.translation.y);
    out.translation.z = a.translation.z + s * (b.translation.z - a.translation.z);

    const Quaternion q0{a.rotation.x, a.rotation.y, a.rotation.z, a.rotation.w};
    const Quaternion q1{b.rotation.x, b.rotation.y, b.rotation.z, b.rotation.w};
    const Quaternion q = slerp(q0, q1, s);

    out.rotation.x = q.x;
    out.rotation.y = q.y;
    out.rotation.z = q.z;
    out.rotation.w = q.w;

    return out;
}

// geometry_msgs::msg::Twist CartesianTrajectoryGenerator::interpolate_twist(
//   const geometry_msgs::msg::Twist & a,
//   const geometry_msgs::msg::Twist & b,
//   double s)
// {
//     geometry_msgs::msg::Twist out;

//     out.linear.x = a.linear.x + s * (b.linear.x - a.linear.x);
//     out.linear.y = a.linear.y + s * (b.linear.y - a.linear.y);
//     out.linear.z = a.linear.z + s * (b.linear.z - a.linear.z);

//     out.angular.x = a.angular.x + s * (b.angular.x - a.angular.x);
//     out.angular.y = a.angular.y + s * (b.angular.y - a.angular.y);
//     out.angular.z = a.angular.z + s * (b.angular.z - a.angular.z);

//     return out;
// }

// TODO: if one ee pose is specified, set the others ee pose to current

bool CartesianTrajectoryGenerator::validate_trajectory(const TrajectoryMsg & msg) const
{
    if (msg.points.empty())
    {
        RCLCPP_WARN(get_node()->get_logger(), "Trajectory has no points");
        return false;
    }

    if (msg.joint_names.size() == 0) {
        // OK
    }
    else if (msg.joint_names.size() == 1) {
        if (msg.joint_names[0] != ee_names_[EEidxL] && msg.joint_names[0] != ee_names_[EEidxR]) {
            RCLCPP_WARN(
            get_node()->get_logger(),
            "Expected joint_names[0] == '%s' or '%s', got '%s'",
            ee_names_[EEidxL].c_str(),
            ee_names_[EEidxR].c_str(),
            msg.joint_names[0].c_str());
            return false;
        }
    }
    else if (msg.joint_names.size() == 2) {
        if (!((msg.joint_names[0] != ee_names_[EEidxL] && msg.joint_names[1] != ee_names_[EEidxR])
                || (msg.joint_names[0] != ee_names_[EEidxR] && msg.joint_names[1] != ee_names_[EEidxL]))) {
            RCLCPP_WARN(
            get_node()->get_logger(),
            "Expected joint_names[0..1] in ['%s', '%s'], got ['%s', '%s']",
            ee_names_[EEidxL].c_str(),
            ee_names_[EEidxR].c_str(),
            msg.joint_names[0].c_str(),
            msg.joint_names[1].c_str());
            return false;
        }
    }
    else {
        RCLCPP_WARN(
        get_node()->get_logger(),
        "Expected zero or one or two MultiDOF joint name, got %zu",
        msg.joint_names.size());
        return false;
    }
    
    double last_time = -std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < msg.points.size(); ++i)
    {
        const auto & point = msg.points[i];

        if (point.transforms.size() != msg.joint_names.size())
        {
            RCLCPP_WARN(
                get_node()->get_logger(),
                "Point %zu must contain %zu transforms, but it has %zu",
                                                i, msg.joint_names.size(), point.transforms.size());
            return false;
        }

        if (!point.velocities.empty()) // && point.velocities.size() != 1)
        {
            RCLCPP_WARN(
                get_node()->get_logger(),
                "Point %zu must contain zero velocity", i);
            return false;
        }

        if (!point.accelerations.empty()) // && point.accelerations.size() != 1)
        {
            RCLCPP_WARN(
                get_node()->get_logger(),
                "Point %zu must contain zero", i);
            return false;
        }

        if (!finite_transform(point.transforms[0]))
        {
            RCLCPP_WARN(get_node()->get_logger(), "Point %zu has non-finite transform[0]", i);
            return false;
        }

        if (point.transforms.size() == 2 && !finite_transform(point.transforms[1]))
        {
            RCLCPP_WARN(get_node()->get_logger(), "Point %zu has non-finite transform[1]", i);
            return false;
        }

        // if (!point.velocities.empty() && !finite_twist(point.velocities[0]))
        // {
        // RCLCPP_WARN(get_node()->get_logger(), "Point %zu has non-finite velocity", i);
        // return false;
        // }

        // if (!point.accelerations.empty() && !finite_twist(point.accelerations[0]))
        // {
        // RCLCPP_WARN(get_node()->get_logger(), "Point %zu has non-finite acceleration", i);
        // return false;
        // }

        const double t = duration_to_seconds(point.time_from_start);
        if (t < 0.0)
        {
            RCLCPP_WARN(get_node()->get_logger(), "Point %zu has negative time_from_start", i);
            return false;
        }

        if (i > 0 && t <= last_time)
        {
            RCLCPP_WARN(get_node()->get_logger(), "Trajectory times must be strictly increasing");
            return false;
        }

        last_time = t;
    }

    return true;
}

// TODO: this function is const
bool CartesianTrajectoryGenerator::sample_trajectory(
    const TrajectoryMsg & msg,
    double t_from_start,
    Sample & sample)
{
    int arm_l_idx = -1;
    int arm_r_idx = -1;
    for (size_t i = 0; i < msg.joint_names.size(); ++i) {
        if (msg.joint_names[i] == ee_names_[EEidxL]) {
            arm_l_idx = int(i);
        }
        else if (msg.joint_names[i] == ee_names_[EEidxR]) {
            arm_r_idx = int(i);
        }
    }

    if (msg.points.empty())
    {
        return false;
    }

    if (t_from_start <= duration_to_seconds(msg.points.front().time_from_start))
    {
        const auto & p = msg.points.front();

        if (arm_l_idx >= 0) {
            sample.transform[EEidxL] = p.transforms[arm_l_idx];
            // sample.twist = p.velocities.empty() ? zero_twist() : p.velocities[0];
            // sample.accel = p.accelerations.empty() ? zero_twist() : p.accelerations[0];
        }
        if (arm_r_idx >= 0) {
            sample.transform[EEidxR] = p.transforms[arm_r_idx];
        }
        dbg_current_traj_pt_ = -2;
        return true;
    }

    if (t_from_start >= duration_to_seconds(msg.points.back().time_from_start))
    {
        const auto & p = msg.points.back();
        if (arm_l_idx >= 0) {
            sample.transform[EEidxL] = p.transforms[arm_l_idx];
            // sample.twist = zero_twist();
            // sample.accel = zero_twist();
        }
        if (arm_r_idx >= 0) {
            sample.transform[EEidxR] = p.transforms[arm_r_idx];
        }
        dbg_current_traj_pt_ = -1;
        return true;
    }

    for (std::size_t i = 0; i + 1 < msg.points.size(); ++i)
    {
        const auto & p0 = msg.points[i];
        const auto & p1 = msg.points[i + 1];

        const double t0 = duration_to_seconds(p0.time_from_start);
        const double t1 = duration_to_seconds(p1.time_from_start);

        if (t_from_start < t0 || t_from_start > t1)
        {
            continue;
        }

        dbg_current_traj_pt_ = i;

        const double dt = t1 - t0;
        const double s = dt > kEps ? std::clamp((t_from_start - t0) / dt, 0.0, 1.0) : 0.0;

        if (arm_l_idx >= 0) {
            sample.transform[EEidxL] = interpolate_transform(p0.transforms[arm_l_idx], p1.transforms[arm_l_idx], s);

            // if (!p0.velocities.empty() && !p1.velocities.empty())
            // {
            //   sample.twist = interpolate_twist(p0.velocities[0], p1.velocities[0], s);
            // }
            // else
            // {
            //   sample.twist = constant_linear_velocity(p0.transforms[0], p1.transforms[0], dt);
            // }

            // if (!p0.accelerations.empty() && !p1.accelerations.empty())
            // {
            //   sample.accel = interpolate_twist(p0.accelerations[0], p1.accelerations[0], s);
            // }
            // else
            // {
            //   sample.accel = zero_twist();
            // }
        }
        if (arm_r_idx >= 0) {
            sample.transform[EEidxR] = interpolate_transform(p0.transforms[arm_r_idx], p1.transforms[arm_r_idx], s);
        }
        return true;
    }

    return false;
}

void CartesianTrajectoryGenerator::write_sample_to_reference_interfaces(const Sample & sample)
{
    reference_interfaces_[EEidxL*REF_COUNT + PX] = sample.transform[EEidxL].translation.x;
    reference_interfaces_[EEidxL*REF_COUNT + PY] = sample.transform[EEidxL].translation.y;
    reference_interfaces_[EEidxL*REF_COUNT + PZ] = sample.transform[EEidxL].translation.z;

    reference_interfaces_[EEidxL*REF_COUNT + QX] = sample.transform[EEidxL].rotation.x;
    reference_interfaces_[EEidxL*REF_COUNT + QY] = sample.transform[EEidxL].rotation.y;
    reference_interfaces_[EEidxL*REF_COUNT + QZ] = sample.transform[EEidxL].rotation.z;
    reference_interfaces_[EEidxL*REF_COUNT + QW] = sample.transform[EEidxL].rotation.w;

    reference_interfaces_[EEidxR*REF_COUNT + PX] = sample.transform[EEidxR].translation.x;
    reference_interfaces_[EEidxR*REF_COUNT + PY] = sample.transform[EEidxR].translation.y;
    reference_interfaces_[EEidxR*REF_COUNT + PZ] = sample.transform[EEidxR].translation.z;

    reference_interfaces_[EEidxR*REF_COUNT + QX] = sample.transform[EEidxR].rotation.x;
    reference_interfaces_[EEidxR*REF_COUNT + QY] = sample.transform[EEidxR].rotation.y;
    reference_interfaces_[EEidxR*REF_COUNT + QZ] = sample.transform[EEidxR].rotation.z;
    reference_interfaces_[EEidxR*REF_COUNT + QW] = sample.transform[EEidxR].rotation.w;

  // reference_interfaces_[VX] = sample.twist.linear.x;
  // reference_interfaces_[VY] = sample.twist.linear.y;
  // reference_interfaces_[VZ] = sample.twist.linear.z;

  // reference_interfaces_[WX] = sample.twist.angular.x;
  // reference_interfaces_[WY] = sample.twist.angular.y;
  // reference_interfaces_[WZ] = sample.twist.angular.z;

  // reference_interfaces_[AX] = sample.accel.linear.x;
  // reference_interfaces_[AY] = sample.accel.linear.y;
  // reference_interfaces_[AZ] = sample.accel.linear.z;

  // reference_interfaces_[ALPHAX] = sample.accel.angular.x;
  // reference_interfaces_[ALPHAY] = sample.accel.angular.y;
  // reference_interfaces_[ALPHAZ] = sample.accel.angular.z;
}

}  // namespace cartesian_trajectory_generator

PLUGINLIB_EXPORT_CLASS(
  cartesian_trajectory_generator::CartesianTrajectoryGenerator,
  controller_interface::ChainableControllerInterface)
