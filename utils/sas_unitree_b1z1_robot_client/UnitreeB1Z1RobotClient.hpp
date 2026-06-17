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
#
# ################################################################
*/


#pragma once
#include "std_msgs/msg/int32_multi_array.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <dqrobotics/DQ.h>
#include <sas_core/sas_clock.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sas_msgs/msg/watchdog_trigger.hpp>
#include <sas_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/battery_state.hpp>


using namespace rclcpp;
using namespace DQ_robotics;

namespace sas
{

class UnitreeB1Z1RobotClient
{
public:
    enum class MODE{
        CONTROL=0,
        MONITORING,
        WATCHDOG
    };
    enum class LEG{
        FR,
        FL,
        RR,
        RL,
    };
    enum class REFERENCE_FRAME
    {
        BODY_FRAME,
        INERTIAL_FRAME,
    };

    enum class HighLevelMode : int {
        IDLE_DEFAULT_STAND = 0,
        FORCED_STAND = 1,
        TARGET_VELOCITY_WALKING = 2,
        PATH_MODE_WALKING = 4,
        POSITION_STAND_DOWN = 5,
        POSITION_STAND_UP = 6,
        DAMPING_MODE = 7,
        RECOVERY_STAND = 9
    };


protected:
    MODE mode_;
    std::string B1_topic_prefix_;
    std::string Z1_topic_prefix_;
    std::shared_ptr<rclcpp::Node> node_;

    //---------- To get data about the robot state and command the robot----------------//
    DQ robot_pose_;
    bool new_robot_pose_data_available_{false};
    bool new_arm_data_available_{false};
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose_state_;
    void _callback_pose_state(const geometry_msgs::msg::PoseStamped& msg);

    DQ robot_odometry_pose_;
    bool new_robot_odometry_pose_data_available_{false};
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_odometry_pose_state_;
    void _callback_odometry_pose_state(const geometry_msgs::msg::PoseStamped& msg);
    //std::shared_ptr<sas::RobotDriverClient> rdi_;

    DQ last_IMU_orientation_when_robot_stopped_;
    bool new_last_IMU_orientation_when_robot_stopped_data_available_{false};
    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_last_IMU_orientation_when_robot_stopped_;
    void _callback_last_IMU_orientation_when_robot_stopped(const geometry_msgs::msg::PoseStamped& msg);

    Vector3d rpy_imu_angles_;
    bool new_robot_rpy_imu_data_available_{false};
    Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_rpy_imu_state_;
    void _callback_rpy_imu_state(const std_msgs::msg::Float64MultiArray& msg);

    Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_target_twist_;
    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_target_holonomic_velocities_;
    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_target_arm_positions_;


    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_stand_commands_;
    Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr   publisher_mode_switch_;


    VectorXd q_arm_;
    VectorXd qFR_;
    VectorXd qFL_;
    VectorXd qRR_;
    VectorXd qRL_;
    Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_FR_joint_states_;
    Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_FL_joint_states_;
    Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_RR_joint_states_;
    Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_RL_joint_states_;
    Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_Z1_joint_states_;

    Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_twist_state_;
    void _callback_subscriber_twist_state(const geometry_msgs::msg::TwistStamped& msg);
    bool new_twist_data_available_{false};
    DQ linear_velocity_from_robot_data_{0};
    DQ angular_velocity_from_robot_data_{0};
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> twist_state_time_point_;

    void _callback_subscriber_battery_state(const sensor_msgs::msg::BatteryState& msg);
    bool new_battery_data_available_{false};
    double battery_status_{0};
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> battery_time_point_;
    Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscriber_battery_state_;

    void _callback_FR_joint_states(const sensor_msgs::msg::JointState& msg);
    void _callback_FL_joint_states(const sensor_msgs::msg::JointState& msg);
    void _callback_RR_joint_states(const sensor_msgs::msg::JointState& msg);
    void _callback_RL_joint_states(const sensor_msgs::msg::JointState& msg);
    void _callback_Z1_joint_states(const sensor_msgs::msg::JointState& msg);


    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> z1_arm_state_time_point_;

    Publisher<sas_msgs::msg::WatchdogTrigger>::SharedPtr publisher_B1_1_watchdog_;
    Publisher<sas_msgs::msg::WatchdogTrigger>::SharedPtr publisher_Z1_1_watchdog_;

    Publisher<sas_msgs::msg::Bool>::SharedPtr publisher_B1_1_shutdown_signal_;
    Publisher<sas_msgs::msg::Bool>::SharedPtr publisher_Z1_1_shutdown_signal_;


    void _set_high_level_mode(const int& mode);


public:
    UnitreeB1Z1RobotClient(const UnitreeB1Z1RobotClient&)=delete;
    UnitreeB1Z1RobotClient()=delete;
    ~UnitreeB1Z1RobotClient();

    UnitreeB1Z1RobotClient(std::shared_ptr<Node>& node,
                           const std::string& B1_topic_prefix,
                           const std::string& Z1_topic_prefix,
                           const MODE& mode);

    bool is_robot_pose_data_available() const;
    bool is_arm_data_available() const;
    bool is_enabled() const;

    VectorXd get_arm_joint_states_including_gripper() const;
    VectorXd get_arm_joint_states() const;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> get_arm_joint_state_time_point() const;

    VectorXd get_leg_joint_states(const LEG& leg);

    DQ get_b1_linear_velocities(const REFERENCE_FRAME& reference_frame) const;
    DQ get_b1_angular_velocities(const REFERENCE_FRAME& reference_frame) const;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> get_twist_state_time_point() const;

    DQ get_b1_pose() const;
    DQ get_b1_odometry_pose() const;
    DQ get_b1_IMU_orientation() const;
    DQ get_b1_last_IMU_orientation_when_robot_stopped() const;
    Vector3d get_rpy_angles_from_IMU() const;
    void set_arm_joint_positions(const VectorXd& target_joint_positions);
    void set_target_b1_planar_joint_velocities(const VectorXd& q_dot_expressed_at_body_frame, const double& deadband=0.032);
    void set_target_b1_twist(const DQ& twist_expressed_at_body_frame, const double& deadband=0.032);
    void send_watchdog_trigger(const double& period_in_seconds = 1.0,
                               const double& maximum_acceptable_delay = 0.5);

    double get_battery_level() const;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> get_battery_time_point() const;

    void send_shutdown_signal();

    std::string get_b1_topic_prefix() const;
    std::string get_z1_topic_prefix() const;

    void set_walking_mode();
    void set_stand_mode();
    void set_forced_stand_commands(const double &roll_angle,
                                   const double &pitch_angle,
                                   const double &yaw_angle,
                                   const double &bodyheight = 0);

};
}
