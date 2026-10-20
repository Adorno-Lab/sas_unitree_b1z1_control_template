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
#   This example is based on https://github.com/MarinhoLab/sas_robot_driver_ur
#
# ################################################################
*/

#include "control_example/control_example.hpp"
#include <dqrobotics_extensions/robot_constraint_manager/numpy.hpp>
#include <dqrobotics_extensions/robot_constraint_manager/robot_constraint_manager.hpp>
#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>


namespace sas
{

using MODE_BLACKLIST_FLAG = RobotDriverClient::MODE_BLACKLIST_FLAG;
using HIGH_LEVEL_MODE = LeggedRobotDriver::HIGH_LEVEL_MODE;

/**
 * @brief ControlExample::ControlExample ctor of the class
 * @param node ROS2 node
 * @param configuration The configuration
 * @param break_loops The atomic boolean to break any loops when the user triggers the interruption signal
 */
ControlExample::ControlExample(std::shared_ptr<Node> &node,
                               const ControlExampleConfiguration &configuration,
                               std::atomic_bool *break_loops)
:
configuration_{configuration},
st_break_loops_{break_loops},
node_{node},
clock_{configuration.thread_sampling_time_sec}
{
    if (configuration_.trunk_to_arm_base_translation.size() != 3)
        throw std::invalid_argument("::trunk_to_arm_base_translation must have 3 elements.");

    // The watchdog is sent by sas_legged_client_window (monitor mode).
    b1_client_ = std::make_shared<LeggedRobotDriverClient>(node_, configuration_.B1_topic_prefix,
                                                           std::vector<MODE_BLACKLIST_FLAG>{MODE_BLACKLIST_FLAG::WATCHDOG_CONTROL});
    z1_client_ = std::make_shared<RobotDriverClient>(node_, configuration_.Z1_topic_prefix,
                                                     std::vector<MODE_BLACKLIST_FLAG>{MODE_BLACKLIST_FLAG::WATCHDOG_CONTROL});
    x_client_ = std::make_unique<ObjectClient>(node_, configuration_.x_prefix);
    xd_client_ = std::make_unique<ObjectClient>(node_, configuration_.xd_prefix);

    subscriber_robot_pose_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        configuration_.B1_topic_prefix + "/get/ekf/robot_pose", 1,
        std::bind(&ControlExample::_callback_robot_pose, this, std::placeholders::_1));
}

ControlExample::~ControlExample()
{

}

void ControlExample::_callback_robot_pose(const geometry_msgs::msg::PoseStamped &msg)
{
    const DQ x = sas::geometry_msgs_pose_stamped_to_dq(msg);
    if (is_unit(x))
        robot_pose_ = x;
    else
        RCLCPP_WARN_STREAM(node_->get_logger(), "::Problem reading the pose of the robot.");
}

bool ControlExample::_should_shutdown() const
{
    return (*st_break_loops_);
}

void ControlExample::_spin_and_sleep()
{
    rclcpp::spin_some(node_);
    clock_.update_and_sleep();
    rclcpp::spin_some(node_);
}

/**
 * @brief ControlExample::_get_configuration returns q = [x, y, phi, q_arm] using the pose of the robot.
 */
VectorXd ControlExample::_get_configuration(const VectorXd &q_arm) const
{
    return DQ_robotics_extensions::Numpy::vstack(
        DQ_robotics_extensions::get_planar_joint_configuration_from_pose(robot_pose_), q_arm);
}

/**
 * @brief ControlExample::_set_b1_mode requests the mode and waits until the B1 is in that mode.
 */
void ControlExample::_set_b1_mode(const HIGH_LEVEL_MODE &mode)
{
    const double timeout_sec = 10.0;
    const double start = clock_.get_elapsed_time_sec();
    double last_request = -1.0;
    while (b1_client_->get_high_level_mode() != mode && !_should_shutdown())
    {
        const double now = clock_.get_elapsed_time_sec();
        if (now - start > timeout_sec)
            throw std::runtime_error("::The B1 did not change its mode.");
        if (last_request < 0.0 || now - last_request > 0.5)
        {
            b1_client_->send_high_level_mode(mode);
            last_request = now;
        }
        _spin_and_sleep();
    }
}

/**
 * @brief ControlExample::_stop_robot stops the base and sets the B1 to IDLE. The Z1 keeps its last target.
 */
void ControlExample::_stop_robot()
{
    b1_client_->send_target_twist(DQ(0));
    b1_client_->send_high_level_mode(HIGH_LEVEL_MODE::IDLE);
    rclcpp::spin_some(node_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "::B1 set to IDLE.");
}

/**
 * @brief ControlExample::control_loop
 */
void ControlExample::control_loop()
{
    clock_.init();

    // Wait for the robot drivers and the pose of the robot
    while ((!b1_client_->is_enabled() || !z1_client_->is_enabled()) && !_should_shutdown())
    {
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for the drivers...");
        _spin_and_sleep();
    }
    while (!is_unit(robot_pose_) && !_should_shutdown())
    {
        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for the pose of the robot (get/ekf/robot_pose)...");
        _spin_and_sleep();
    }
    if (_should_shutdown())
        return;
    RCLCPP_INFO_STREAM(node_->get_logger(), "::Robot is ready!");

    /*********************************************
     * ROBOT KINEMATIC MODEL
     * *******************************************/
    const auto& t = configuration_.trunk_to_arm_base_translation;
    robot_model_ = std::make_shared<UnitreeB1Z1MobileRobot>();
    robot_model_->update_base_offset(1 + 0.5*E_*(t.at(0)*i_ + t.at(1)*j_ + t.at(2)*k_));
    robot_model_->update_base_height_from_IMU(robot_pose_);

    /// The Z1 driver has 7 joints: the 6 joints of the arm, and the gripper.
    const VectorXd q_z1 = z1_client_->get_joint_positions();
    VectorXd qi_arm = q_z1.head(6);
    const VectorXd q_gripper = q_z1.tail(1);

    // The desired frame starts at the current pose of the end-effector
    xd_client_->send_pose(robot_model_->fkm(_get_configuration(qi_arm)));

    _set_b1_mode(HIGH_LEVEL_MODE::WALKING);
    RCLCPP_INFO_STREAM(node_->get_logger(), "::B1 in WALKING.");

    //###########################################################################################################//
    //####################### Main Kinematic control loop running here ##########################################//
    const double w = 0.5;
    const double& T = configuration_.thread_sampling_time_sec;
    unsigned int i = 0;
    while(!_should_shutdown())
    {
        _spin_and_sleep();
        RCLCPP_INFO_ONCE(node_->get_logger(), "Control loop!");

        robot_model_->update_base_height_from_IMU(robot_pose_);
        const VectorXd q = _get_configuration(qi_arm);
        const DQ x = robot_model_->fkm(q);
        x_client_->send_pose(x);

        /// The desired pose is not used in this example. If the frame xd is not available, xd = x.
        [[maybe_unused]] const DQ xd = xd_client_->is_enabled() ? xd_client_->get_pose() : x;

        const double t = i*T;
        double phi_dot = 0.04*sin(w*t);
        if (t > 20.0)
            phi_dot = 0.0;

        /// The twist of the base is expressed in the body frame: wz*k_ + E_*(vx*i_ + vy*j_).
        /// If your controller computes the planar velocities [x_dot, y_dot, phi_dot] in the inertial frame (u_qdot), use
        ///  VectorXd u = DQ_robotics_extensions::get_planar_joint_configuration_velocities_at_body_frame(robot_pose_, u_qdot.head(3));
        const double vx = 0.0;
        const double vy = 0.0;
        b1_client_->send_target_twist(phi_dot*k_ + E_*(vx*i_ + vy*j_));

        qi_arm(0) = (pi/4)*sin(w*t);

        /// The Z1 driver expects 7 positions: the 6 joints of the arm and the gripper.
        z1_client_->send_target_joint_positions(DQ_robotics_extensions::Numpy::vstack(qi_arm, q_gripper));
        i++;
    }

    _stop_robot();
}


}
