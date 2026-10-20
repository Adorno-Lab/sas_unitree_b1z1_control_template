/*
#    Copyright (c) 2025-2026 Adorno-Lab
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License.
#    If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana (email: juanjose.quirozomana@manchester.ac.uk)
#   Based on sas_extended_kalman_filter_unitree_b1 (rap2025_wholebodycontrol)
#
# ################################################################
*/

#include "sas_unitree_b1_ekf/sas_unitree_b1_ekf.hpp"
#include <algorithm>
#include <cmath>
#include <tf2/exceptions.h>
#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>

namespace sas
{

using MODE_BLACKLIST_FLAG = RobotDriverClient::MODE_BLACKLIST_FLAG;

UnitreeB1EKF::UnitreeB1EKF(const std::shared_ptr<rclcpp::Node> &node,
                           const UnitreeB1EKFConfiguration &configuration,
                           std::atomic_bool *break_loops)
    :node_{node}, configuration_{configuration}, break_loops_{break_loops},
    clock_{configuration.thread_sampling_time_sec}
{
    if (configuration_.rear_marker_to_trunk_translation.size() != 3 ||
        configuration_.process_noise.size() != 3 ||
        configuration_.measurement_noise.size() != 3)
        throw std::invalid_argument("::rear_marker_to_trunk_translation, process_noise, and measurement_noise must have 3 elements.");

    // The EKF only reads the B1.
    b1_ = std::make_shared<LeggedRobotDriverClient>(node_, configuration_.b1_topic_prefix,
                                                    std::vector<MODE_BLACKLIST_FLAG>{MODE_BLACKLIST_FLAG::JOINT_CONTROL,
                                                                                     MODE_BLACKLIST_FLAG::WATCHDOG_CONTROL});

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_robot_pose_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        configuration_.b1_topic_prefix + "/get/ekf/robot_pose", 1);

    rear_.name = configuration_.robot_vicon_marker_rear;
    front_.name = configuration_.robot_vicon_marker_front;

    const auto& t = configuration_.rear_marker_to_trunk_translation;
    x_rear_to_trunk_ = 1 + 0.5*E_*(t.at(0)*i_ + t.at(1)*j_ + t.at(2)*k_);

    state_.setZero();
    SIGMA_ = 0.01*Eigen::Matrix3d::Identity();
    R_ = Eigen::Vector3d(configuration_.process_noise.at(0),
                         configuration_.process_noise.at(1),
                         configuration_.process_noise.at(2)).asDiagonal();
    Q_ = Eigen::Vector3d(configuration_.measurement_noise.at(0),
                         configuration_.measurement_noise.at(1),
                         configuration_.measurement_noise.at(2)).asDiagonal();
}

bool UnitreeB1EKF::_should_shutdown() const
{
    return (*break_loops_);
}

void UnitreeB1EKF::_spin_and_sleep()
{
    rclcpp::spin_some(node_);
    clock_.update_and_sleep();
}

DQ UnitreeB1EKF::_transform_to_dq(const geometry_msgs::msg::TransformStamped &msg)
{
    const DQ r = normalize(DQ(msg.transform.rotation.w,
                              msg.transform.rotation.x,
                              msg.transform.rotation.y,
                              msg.transform.rotation.z));
    const DQ t = msg.transform.translation.x*i_ + msg.transform.translation.y*j_ + msg.transform.translation.z*k_;
    return normalize(r + 0.5*E_*t*r);
}

/**
 * @brief Yaw of the pose x, from the projection of its x-axis on the ground.
 */
double UnitreeB1EKF::_get_yaw(const DQ &x)
{
    const DQ r = x.P();
    const Eigen::VectorXd x_axis = (r*i_*r.conj()).vec3();
    return std::atan2(x_axis(1), x_axis(0));
}

/**
 * @brief Wraps the angle to [-pi, pi).
 */
double UnitreeB1EKF::_wrap_angle(const double &angle)
{
    return std::remainder(angle, 2.0*M_PI);
}

/**
 * @brief Incremental mean of unit dual quaternions. n is the number of samples, including x.
 */
DQ UnitreeB1EKF::_running_mean(const DQ &mean, const DQ &x, const int &n)
{
    if (n <= 1)
        return x;
    DQ error = mean.conj()*x;
    if (error.vec8()(0) < 0) // double cover
        error = -error;
    return normalize(mean*pow(error, 1.0/n));
}

/**
 * @brief Reads the marker from tf. A new sample is accepted if its stamp changed and the marker is not too tilted
 *        (Vicon sometimes gives wrong poses near the borders of the workspace).
 */
void UnitreeB1EKF::_update_marker(Marker &marker)
{
    try
    {
        const auto msg = tf_buffer_->lookupTransform(configuration_.world_frame, marker.name, tf2::TimePointZero);
        const rclcpp::Time stamp(msg.header.stamp, RCL_ROS_TIME);
        if (stamp != marker.stamp)
        {
            marker.stamp = stamp;
            const DQ x = _transform_to_dq(msg);
            const DQ r = x.P();
            const double cos_tilt = std::clamp((r*k_*r.conj()).vec3()(2), -1.0, 1.0);
            if (std::acos(cos_tilt) < configuration_.max_marker_tilt_rad)
            {
                marker.pose = x;
                marker.new_sample = true;
                marker.last_update_time_sec = clock_.get_elapsed_time_sec();
            }
        }
    }
    catch (const tf2::TransformException&)
    {
        // Not published yet. It is handled by the timeout.
    }

    const bool valid = marker.last_update_time_sec >= 0.0 &&
                       clock_.get_elapsed_time_sec() - marker.last_update_time_sec < configuration_.marker_timeout_sec;
    if (valid != marker.valid)
    {
        if (valid)
            RCLCPP_INFO_STREAM(node_->get_logger(), "::Marker " << marker.name << " OK.");
        else
            RCLCPP_WARN_STREAM(node_->get_logger(), "::Marker " << marker.name << " lost.");
    }
    marker.valid = valid;
}

void UnitreeB1EKF::_update_markers()
{
    const bool any_valid = rear_.valid || front_.valid;
    _update_marker(rear_);
    _update_marker(front_);
    if (any_valid && !rear_.valid && !front_.valid)
        RCLCPP_WARN_STREAM(node_->get_logger(), "::Both markers lost. Only the measured twist is used.");
}

/**
 * @brief Gets the pose of the trunk from the markers, if there are new samples. With both markers, it is the mean
 *        of the two (the front one moved to the rear with x_rear_to_front_).
 * @param x_trunk The pose of the trunk.
 * @return True if there is a new measurement.
 */
bool UnitreeB1EKF::_get_measurement(DQ &x_trunk)
{
    const bool new_sample = rear_.new_sample || front_.new_sample;
    rear_.new_sample = false;
    front_.new_sample = false;
    if (!new_sample)
        return false;

    DQ x_rear;
    if (rear_.valid && front_.valid)
    {
        const DQ x_rear_from_front = front_.pose*x_rear_to_front_.conj();
        x_rear = _running_mean(_running_mean(DQ(1), rear_.pose, 1), x_rear_from_front, 2);
    }
    else if (rear_.valid)
        x_rear = rear_.pose;
    else if (front_.valid)
        x_rear = front_.pose*x_rear_to_front_.conj();
    else
        return false;

    x_trunk = x_rear*x_rear_to_trunk_;
    return true;
}

/**
 * @brief Integrates the measured twist (body frame) and propagates the covariance.
 */
void UnitreeB1EKF::_prediction_step(const double &dt)
{
    const DQ twist = b1_->get_measured_twist();
    const double phi = state_(2);
    const DQ r = cos(phi/2.0) + k_*sin(phi/2.0);
    const Eigen::VectorXd v_body = twist.D().vec3();
    const Eigen::VectorXd v_world = (r*twist.D()*r.conj()).vec3();
    const double wz = twist.P().vec3()(2);

    Eigen::Matrix3d G = Eigen::Matrix3d::Identity();
    G(0, 2) = -dt*(v_body(0)*sin(phi) + v_body(1)*cos(phi));
    G(1, 2) =  dt*(v_body(0)*cos(phi) - v_body(1)*sin(phi));

    state_(0) += v_world(0)*dt;
    state_(1) += v_world(1)*dt;
    state_(2) = _wrap_angle(phi + wz*dt);
    SIGMA_ = G*SIGMA_*G.transpose() + R_;
}

/**
 * @brief Corrects the state with the pose of the trunk measured with Vicon (H = I). Also updates the height.
 */
void UnitreeB1EKF::_correction_step(const DQ &x_trunk)
{
    const Eigen::VectorXd t = x_trunk.translation().vec3();
    height_ = t(2);

    const Eigen::Vector3d z(t(0), t(1), _get_yaw(x_trunk));
    Eigen::Vector3d innovation = z - state_;
    innovation(2) = _wrap_angle(innovation(2));

    const Eigen::Matrix3d K = SIGMA_*(SIGMA_ + Q_).inverse();
    state_ += K*innovation;
    state_(2) = _wrap_angle(state_(2));
    SIGMA_ = (Eigen::Matrix3d::Identity() - K)*SIGMA_;
}

DQ UnitreeB1EKF::_get_estimated_pose() const
{
    const DQ r = cos(state_(2)/2.0) + k_*sin(state_(2)/2.0);
    const DQ t = state_(0)*i_ + state_(1)*j_ + height_*k_;
    return r + 0.5*E_*t*r;
}

void UnitreeB1EKF::_publish_estimated_pose()
{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = configuration_.world_frame;
    msg.pose = dq_to_geometry_msgs_pose(_get_estimated_pose());
    publisher_robot_pose_->publish(msg);
}

/**
 * @brief Waits for the driver and both markers. The initial state and x_rear_to_front_ are the mean of
 *        initialization_samples samples.
 */
void UnitreeB1EKF::_initialize()
{
    while (!b1_->is_enabled() && !_should_shutdown())
    {
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for the B1 driver on " << configuration_.b1_topic_prefix);
        _spin_and_sleep();
    }
    if (_should_shutdown())
        return;
    if (!b1_->is_supported(LeggedRobotDriver::LEGGED_FUNCTIONALITY::MEASURED_TWIST))
        throw std::runtime_error("::The B1 driver does not publish the measured twist (get/twist).");

    bool twist_received = false;
    while (!twist_received && !_should_shutdown())
    {
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for the measured twist.");
        _spin_and_sleep();
        try
        {
            b1_->get_measured_twist();
            twist_received = true;
        }
        catch (const std::runtime_error&)
        {
        }
    }

    DQ x_rear{1};
    DQ x_rear_to_front{1};
    int n = 0;
    while (n < configuration_.initialization_samples && !_should_shutdown())
    {
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for the markers " << rear_.name << " and " << front_.name);
        _spin_and_sleep();
        _update_markers();
        if (rear_.valid && front_.valid && (rear_.new_sample || front_.new_sample))
        {
            rear_.new_sample = false;
            front_.new_sample = false;
            n++;
            x_rear = _running_mean(x_rear, rear_.pose, n);
            x_rear_to_front = _running_mean(x_rear_to_front, rear_.pose.conj()*front_.pose, n);
        }
    }
    if (_should_shutdown())
        return;

    x_rear_to_front_ = x_rear_to_front;
    const DQ x_trunk = x_rear*x_rear_to_trunk_;
    const Eigen::VectorXd t = x_trunk.translation().vec3();
    height_ = t(2);
    state_ << t(0), t(1), _get_yaw(x_trunk);
    RCLCPP_INFO_STREAM(node_->get_logger(), "::EKF initialized at x = " << state_(0) << ", y = " << state_(1)
                                            << ", phi = " << state_(2));
}

void UnitreeB1EKF::loop()
{
    clock_.init();
    _initialize();

    double last_time = clock_.get_elapsed_time_sec();
    while (!_should_shutdown())
    {
        _spin_and_sleep();
        const double time = clock_.get_elapsed_time_sec();
        _prediction_step(time - last_time);
        last_time = time;

        _update_markers();
        DQ x_trunk;
        if (_get_measurement(x_trunk))
            _correction_step(x_trunk);

        _publish_estimated_pose();
    }
}

}
