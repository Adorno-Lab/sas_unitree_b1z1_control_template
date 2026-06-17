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


#include <sas_conversions/DQ_geometry_msgs_conversions.hpp>
#include <sas_unitree_b1z1_robot_client/UnitreeB1Z1RobotClient.hpp>
#include <sas_core/eigen3_std_conversions.hpp>


namespace sas
{
/**
 * @brief UnitreeB1Z1RobotClient::UnitreeB1Z1RobotClient ctor of the class
 * @param node The ros node
 * @param B1_topic_prefix The prefix of the topic used in the SAS driver for the Unitree B1
 * @param Z1_topic_prefix The prefix of the topic used in the SAS driver for the Unitree Z1
 * @param mode The mode of the driver. You can have serveral client as monitors, but only one as control.
 */
UnitreeB1Z1RobotClient::UnitreeB1Z1RobotClient(std::shared_ptr<Node>& node,
                                               const std::string& B1_topic_prefix,
                                               const std::string& Z1_topic_prefix,
                                               const MODE& mode)
    :mode_{mode}, B1_topic_prefix_{B1_topic_prefix}, Z1_topic_prefix_{Z1_topic_prefix}, node_{node}
{
    std::string message = "::UnitreeB1Z1RobotClient with /" + B1_topic_prefix_ + " and /" + Z1_topic_prefix_;
    RCLCPP_INFO_ONCE(node_->get_logger(), "%s", message.c_str());

    subscriber_battery_state_ =  node_->create_subscription<sensor_msgs::msg::BatteryState>(
        B1_topic_prefix_ + "/get/battery_state", 1,
        std::bind(&UnitreeB1Z1RobotClient::_callback_subscriber_battery_state,
                  this, std::placeholders::_1)
        );

    subscriber_pose_state_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        B1_topic_prefix_ + "/get/ekf/robot_pose", 1,
        std::bind(&UnitreeB1Z1RobotClient::_callback_pose_state,
                  this, std::placeholders::_1)
        );

    subscriber_odometry_pose_state_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        B1_topic_prefix_ + "/get/pose_state", 1,
        std::bind(&UnitreeB1Z1RobotClient::_callback_odometry_pose_state,
                  this, std::placeholders::_1)
        );

    subscriber_last_IMU_orientation_when_robot_stopped_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        B1_topic_prefix_ + "/get/last_imu_orientation_when_robot_stopped", 1,
        std::bind(&UnitreeB1Z1RobotClient::_callback_last_IMU_orientation_when_robot_stopped,
                  this, std::placeholders::_1)
        );

    subscriber_rpy_imu_state_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        B1_topic_prefix_ + "/get/rpy_angles", 1,
        std::bind(&UnitreeB1Z1RobotClient::_callback_rpy_imu_state,
                  this, std::placeholders::_1)
        );

    subscriber_FR_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        B1_topic_prefix_  + "/get/FR_joint_states", 1, std::bind(&UnitreeB1Z1RobotClient::_callback_FR_joint_states, this, std::placeholders::_1)
        );
    subscriber_FL_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        B1_topic_prefix_  + "/get/FL_joint_states", 1, std::bind(&UnitreeB1Z1RobotClient::_callback_FL_joint_states, this, std::placeholders::_1)
        );
    subscriber_RR_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        B1_topic_prefix_  + "/get/RR_joint_states", 1, std::bind(&UnitreeB1Z1RobotClient::_callback_RR_joint_states, this, std::placeholders::_1)
        );
    subscriber_RL_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        B1_topic_prefix_  + "/get/RL_joint_states", 1, std::bind(&UnitreeB1Z1RobotClient::_callback_RL_joint_states, this, std::placeholders::_1)
        );

    subscriber_Z1_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        Z1_topic_prefix_+ "/get/joint_states", 1, std::bind(&UnitreeB1Z1RobotClient::_callback_Z1_joint_states,
                  this, std::placeholders::_1)
        );


    subscriber_twist_state_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        B1_topic_prefix_ + "/get/twist_state",
        1,
        std::bind(&UnitreeB1Z1RobotClient::_callback_subscriber_twist_state,
                  this, std::placeholders::_1)
        );

    publisher_B1_1_shutdown_signal_ = node->create_publisher<sas_msgs::msg::Bool>(B1_topic_prefix_  + "/set/shutdown", 1);
    publisher_Z1_1_shutdown_signal_ = node->create_publisher<sas_msgs::msg::Bool>(Z1_topic_prefix_  + "/set/shutdown", 1);

    if (mode_ == MODE::CONTROL)
    {
        publisher_target_arm_positions_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            Z1_topic_prefix_ + "/set/target_joint_positions", 1);

        publisher_target_holonomic_velocities_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            B1_topic_prefix_ + "/set/holonomic_target_velocities", 1);

        publisher_target_twist_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
            B1_topic_prefix_ + "/set/target_twist", 1);

        publisher_stand_commands_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            B1_topic_prefix_ + "/set/stand_commands",1);

        publisher_mode_switch_    = node_->create_publisher<std_msgs::msg::Int32MultiArray>(
            B1_topic_prefix_ + "/set/mode",1
            );
    }
    if (mode_ == MODE::WATCHDOG)
    {
        publisher_B1_1_watchdog_ = node_->create_publisher<sas_msgs::msg::WatchdogTrigger>(B1_topic_prefix_ + "/set/watchdog_trigger", 1);
        publisher_Z1_1_watchdog_ = node_->create_publisher<sas_msgs::msg::WatchdogTrigger>(Z1_topic_prefix_ + "/set/watchdog_trigger", 1);
    }

    //rdi_ = std::make_shared<sas::RobotDriverClient>(node_, Z1_topic_prefix_);
}

/**
 * @brief UnitreeB1Z1RobotClient::get_b1_pose gets the pose of the B1 robot
 * @return the desired pose
 */
DQ UnitreeB1Z1RobotClient::get_b1_pose() const
{
    return robot_pose_;
}

/**
 * @brief UnitreeB1Z1RobotClient::get_b1_odometry_pose
 * @return
 */
DQ UnitreeB1Z1RobotClient::get_b1_odometry_pose() const
{
    return robot_odometry_pose_;
}

/**
 * @brief UnitreeB1Z1RobotClient::get_b1_IMU_orientation
 * @return
 */
DQ UnitreeB1Z1RobotClient::get_b1_IMU_orientation() const
{
    return robot_odometry_pose_.rotation();
}


/**
 * @brief UnitreeB1Z1RobotClient::get_b1_last_IMU_orientation_when_robot_stopped
 * @return
 */
DQ UnitreeB1Z1RobotClient::get_b1_last_IMU_orientation_when_robot_stopped() const
{
    return last_IMU_orientation_when_robot_stopped_.rotation();
}

Vector3d UnitreeB1Z1RobotClient::get_rpy_angles_from_IMU() const
{
    return rpy_imu_angles_;
}

/**
 * @brief UnitreeB1Z1RobotClient::_callback_pose_state
 * @param msg
 */
void UnitreeB1Z1RobotClient::_callback_pose_state(const geometry_msgs::msg::PoseStamped& msg)
{
    robot_pose_ =   sas::geometry_msgs_pose_stamped_to_dq(msg);
    new_robot_pose_data_available_ = true;
}


void UnitreeB1Z1RobotClient::_callback_odometry_pose_state(const geometry_msgs::msg::PoseStamped& msg)
{
    robot_odometry_pose_ = sas::geometry_msgs_pose_stamped_to_dq(msg);
    new_robot_odometry_pose_data_available_ = true;
}

void UnitreeB1Z1RobotClient::_callback_last_IMU_orientation_when_robot_stopped(const geometry_msgs::msg::PoseStamped& msg)
{
    last_IMU_orientation_when_robot_stopped_= sas::geometry_msgs_pose_stamped_to_dq(msg);
    new_last_IMU_orientation_when_robot_stopped_data_available_ = true;
}

void UnitreeB1Z1RobotClient::_callback_rpy_imu_state(const std_msgs::msg::Float64MultiArray &msg)
{
    new_robot_rpy_imu_data_available_ = true;
    rpy_imu_angles_ = Eigen::Map<const Eigen::Vector3d>(msg.data.data());
}

/**
 * @brief UnitreeB1Z1RobotClient::is_robot_pose_data_available checks if the data from B1 SAS driver
 *          is available.
 * @return True if the data is available. False otherwise.
 */
bool UnitreeB1Z1RobotClient::is_robot_pose_data_available() const
{
    return new_robot_pose_data_available_ || new_robot_odometry_pose_data_available_;
}

/**
 * @brief UnitreeB1Z1RobotClient::is_arm_data_available checks if the data from Z1 SAS driver
 *          is available.
 * @return True if the data is available. False otherwise.
 */
bool UnitreeB1Z1RobotClient::is_arm_data_available() const
{
    return new_arm_data_available_;
    //return rdi_->is_enabled();
}


/**
 * @brief UnitreeB1Z1RobotClient::is_enabled checks if the data for both drivers (B1 and Z1) are available.
 * @return True if the data is available. False otherwise.
 */
bool UnitreeB1Z1RobotClient::is_enabled() const
{
    return is_arm_data_available() && is_robot_pose_data_available();
}

/**
 * @brief UnitreeB1Z1RobotClient::get_arm_joint_states gets the joint positions of the Z1 arm.
 *          The gripper position is not included.
 * @return The joint positions of the Z1 arm.
 */
VectorXd  UnitreeB1Z1RobotClient::get_arm_joint_states() const
{
    //VectorXd q = rdi_->get_joint_positions();
    //return q.head(6);
    return q_arm_.head(6);
}

/**
 * @brief UnitreeB1Z1RobotClient::get_arm_joint_states_including_gripper gets the joint positions of the Z1 arm
 *          including the gripper.
 * @return The joint positions of the Z1 arm and its gripper.
 */
VectorXd UnitreeB1Z1RobotClient::get_arm_joint_states_including_gripper() const
{
    //return rdi_->get_joint_positions();
    return q_arm_;
}

/**
 * @brief UnitreeB1Z1RobotClient::_callback_subscriber_twist_state
 * @param msg
 */
void UnitreeB1Z1RobotClient::_callback_subscriber_twist_state(const geometry_msgs::msg::TwistStamped& msg)
{
    angular_velocity_from_robot_data_ = msg.twist.angular.x*i_ + msg.twist.angular.y*j_ + msg.twist.angular.z*k_;
    linear_velocity_from_robot_data_  =  msg.twist.linear.x*i_  + msg.twist.linear.y*j_  + msg.twist.linear.z*k_;

    twist_state_time_point_ = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>(
        std::chrono::seconds(msg.header.stamp.sec) + std::chrono::nanoseconds(msg.header.stamp.nanosec)
        );
    new_twist_data_available_ = true;
}

/**
 * @brief UnitreeB1Z1RobotClient::get_b1_linear_velocities gets the linear velocities of the B1 robot
 * @param reference_frame The desired reference frame.
 * @return The desired linear velocities.
 */
DQ UnitreeB1Z1RobotClient::get_b1_linear_velocities(const REFERENCE_FRAME& reference_frame) const
{
    if (reference_frame == REFERENCE_FRAME::INERTIAL_FRAME)
    {
        // Express the velocities in the world frame
        const DQ& r = get_b1_pose().P();
        //DQ angular_velocity_world_frame = r*angular_velocity_from_robot_data_*r.conj();
        const DQ linear_velocity_world_frame =  r*linear_velocity_from_robot_data_*r.conj();
        return linear_velocity_world_frame;
    }
    else // BODY FRAME
        return linear_velocity_from_robot_data_;

}

/**
 * @brief UnitreeB1Z1RobotClient::get_b1_angular_velocities gets the angular velocities of the B1 robot
 * @param reference_frame The desired reference frame.
 * @return The desired angular velocities.
 */
DQ UnitreeB1Z1RobotClient::get_b1_angular_velocities(const REFERENCE_FRAME& reference_frame) const
{
    if (reference_frame == REFERENCE_FRAME::INERTIAL_FRAME)
    {
        // Express the velocities in the world frame
        const DQ& r = get_b1_pose().P();
        const DQ angular_velocity_world_frame = r*angular_velocity_from_robot_data_*r.conj();
        return angular_velocity_world_frame;
    }
    else // BODY FRAME
        return angular_velocity_from_robot_data_;
}

/**
 * @brief UnitreeB1Z1RobotClient::set_arm_joint_positions sets the joint positions of the arm, including the gripper.
 * @param target_joint_positions The desired target joint positions including the gripper. The gripper is closed with 0.0 and
 *        open with a negative value (radian angle).
 */
void UnitreeB1Z1RobotClient::set_arm_joint_positions(const VectorXd &target_joint_positions)
{
    if (mode_ == MODE::CONTROL && publisher_target_arm_positions_)
    {
        //rdi_->send_target_joint_positions(target_joint_positions);
        std_msgs::msg::Float64MultiArray ros_msg_u_arm_commands;
        ros_msg_u_arm_commands.data = sas::vectorxd_to_std_vector_double(target_joint_positions);
        publisher_target_arm_positions_->publish(ros_msg_u_arm_commands);
    }
    else
        RCLCPP_ERROR_STREAM_ONCE(node_->get_logger(), std::string("::Warning:: UnitreeB1Z1RobotClient::set_arm_joint_positions "
                                                                  "is available only in CONTROL mode"));
}

/**
 * @brief UnitreeB1Z1RobotClient::set_target_b1_planar_joint_velocities set the target planar joint velocities (x_dot, y_dot, phi_dot)
 *                  of the B1 robot.
 * @param q_dot_expressed_at_body_frame The target planar joint velocities expressed at the body frame.
 * @param deadband The desired deadband. Target velocities below this dead band will be set to zero.
 */
void UnitreeB1Z1RobotClient::set_target_b1_planar_joint_velocities(const VectorXd &q_dot_expressed_at_body_frame,
                                                                   const double &deadband)
{
    if (mode_ == MODE::CONTROL && publisher_target_holonomic_velocities_)
    {
        VectorXd u_base = q_dot_expressed_at_body_frame;
        // The B1 robot does not move with velocities below 0.03, but the legs continue to move.
        // Therefore, we enforce a deadband.
        //const double min_vel = 0.032;
        for (int i=0;i<u_base.size();i++)
        {
            if (std::abs(u_base(i))<=deadband)
                u_base(i) = 0.0;
        }
        std_msgs::msg::Float64MultiArray ros_msg_u_base;
        ros_msg_u_base.data = sas::vectorxd_to_std_vector_double(u_base);
        publisher_target_holonomic_velocities_->publish(ros_msg_u_base);
    }else
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(), std::string("::Warning:: UnitreeB1Z1RobotClient::set_target_B1_planar_joint_velocities "
                                                                  "is available only in CONTROL mode"));
    }
}


void UnitreeB1Z1RobotClient::set_target_b1_twist(const DQ& twist_expressed_at_body_frame, const double &deadband)
{
    if (mode_ == MODE::CONTROL && publisher_target_twist_)
    {
        VectorXd u_base = twist_expressed_at_body_frame.vec6();
        // The B1 robot does not move with velocities below 0.03, but the legs continue to move.
        // Therefore, we enforce a deadband.
        //const double min_vel = 0.032;
        for (int i=0;i<u_base.size();i++)
        {
            if (std::abs(u_base(i))<=deadband)
                u_base(i) = 0.0;
        }
        geometry_msgs::msg::TwistStamped msg;
        msg.twist.angular.x = u_base(0);
        msg.twist.angular.y = u_base(1);
        msg.twist.angular.z = u_base(2);
        msg.twist.linear.x  = u_base(3);
        msg.twist.linear.y  = u_base(4);
        msg.twist.linear.z  = u_base(5);
        publisher_target_twist_->publish(msg);
    }else
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(), std::string("::Warning:: UnitreeB1Z1RobotClient::set_target_b1_twist "
                                                             "is available only in CONTROL mode"));
    }
}
    //publisher_target_twist_


/**
 * @brief UnitreeB1Z1RobotClient::get_leg_joint_states returns the joint positions of the desired leg kinematic chain
 * @param leg The desired leg
 * @return The joint positions of the desired leg kinematic chain
 */
VectorXd UnitreeB1Z1RobotClient::get_leg_joint_states(const LEG& leg)
{
    switch (leg) {

    case LEG::FR:
        return qFR_;
    case LEG::FL:
        return qFL_;
    case LEG::RR:
        return qRR_;
    case LEG::RL:
        return qRL_;
    default:
        throw std::runtime_error("Wrong leg");
    }
}

/**
 * @brief UnitreeB1Z1RobotClient::get_battery_level gets the battery level
 * @return The desired battery level.
 */
double UnitreeB1Z1RobotClient::get_battery_level() const
{
    return battery_status_;
}


/**
 * @brief UnitreeB1Z1RobotClient::_callback_FR_joint_states
 * @param msg
 */
void UnitreeB1Z1RobotClient::_callback_FR_joint_states(const sensor_msgs::msg::JointState& msg)
{
    qFR_ = std_vector_double_to_vectorxd(msg.position);
}

/**
 * @brief UnitreeB1Z1RobotClient::_callback_FL_joint_states
 * @param msg
 */
void UnitreeB1Z1RobotClient::_callback_FL_joint_states(const sensor_msgs::msg::JointState& msg)
{
    qFL_ = std_vector_double_to_vectorxd(msg.position);
}

/**
 * @brief UnitreeB1Z1RobotClient::_callback_RR_joint_states
 * @param msg
 */
void UnitreeB1Z1RobotClient::_callback_RR_joint_states(const sensor_msgs::msg::JointState& msg)
{
    qRR_ = std_vector_double_to_vectorxd(msg.position);
}

/**
 * @brief UnitreeB1Z1RobotClient::_callback_RL_joint_states
 * @param msg
 */
void UnitreeB1Z1RobotClient::_callback_RL_joint_states(const sensor_msgs::msg::JointState& msg)
{
    qRL_ = std_vector_double_to_vectorxd(msg.position);
}

/**
 * @brief UnitreeB1Z1RobotClient::_callback_Z1_joint_states
 * @param msg
 */
void UnitreeB1Z1RobotClient::_callback_Z1_joint_states(const sensor_msgs::msg::JointState& msg)
{
    q_arm_ = std_vector_double_to_vectorxd(msg.position);
    z1_arm_state_time_point_ = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>(
        std::chrono::seconds(msg.header.stamp.sec) + std::chrono::nanoseconds(msg.header.stamp.nanosec)
        );
    new_arm_data_available_ = true;
}

/**
 * @brief UnitreeB1Z1RobotClient::send_watchdog_trigger send the watchdog signal. The watchdog will be enabled on
 *             the SAS robot drivers when the first watchdog signal is received.
 * @param period_in_seconds The desired period in seconds for the watchdog.
 * @param maximum_acceptable_delay The maximum acceptable delay. If you have unsynchronized clocks between
 *        the B1 computer and the computer that is sending the watchdog, you can define a huge value here.
 */
void UnitreeB1Z1RobotClient::send_watchdog_trigger(const double& period_in_seconds,
                                                   const double& maximum_acceptable_delay)
{
    sas_msgs::msg::WatchdogTrigger ros_msg;
    ros_msg.header = std_msgs::msg::Header();
    ros_msg.header.stamp = rclcpp::Clock().now();
    ros_msg.status = true;
    ros_msg.period_in_seconds = period_in_seconds;
    ros_msg.maximum_acceptable_delay_in_seconds = maximum_acceptable_delay;

    publisher_B1_1_watchdog_->publish(ros_msg);
    publisher_Z1_1_watchdog_->publish(ros_msg);

}


/**
 * @brief UnitreeB1Z1RobotClient::send_shutdown_signal sends a shutdown signal to stop the robot
 */
void UnitreeB1Z1RobotClient::send_shutdown_signal()
{
    sas_msgs::msg::Bool ros_msg;
    ros_msg.data = true;
    publisher_B1_1_shutdown_signal_->publish(ros_msg);
    publisher_Z1_1_shutdown_signal_->publish(ros_msg);
}

/**
 * @brief UnitreeB1Z1RobotClient::get_b1_topic_prefix gets the B1 topic prefix
 * @return the B1 topic prefix
 */
std::string UnitreeB1Z1RobotClient::get_b1_topic_prefix() const
{
    return B1_topic_prefix_;
}

/**
 * @brief UnitreeB1Z1RobotClient::get_z1_topic_prefix gets the Z1 topic prefix
 * @return the Z1 topic prefix
 */
std::string UnitreeB1Z1RobotClient::get_z1_topic_prefix() const
{
    return Z1_topic_prefix_;
}

/**
 * @brief UnitreeB1Z1RobotClient::_callback_subscriber_battery_state
 * @param msg
 */
void UnitreeB1Z1RobotClient::_callback_subscriber_battery_state(const sensor_msgs::msg::BatteryState& msg)
{
    battery_status_ = msg.percentage;
    battery_time_point_ = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>(
        std::chrono::seconds(msg.header.stamp.sec) + std::chrono::nanoseconds(msg.header.stamp.nanosec)
        );
    new_battery_data_available_ = true;
}

/**
 * @brief UnitreeB1Z1RobotClient::get_battery_time_point gets the time point associated with the last battery level message
 *      received.
 * @return The desired time point.
 */
std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> UnitreeB1Z1RobotClient::get_battery_time_point() const
{
    return battery_time_point_;
}

/**
 * @brief UnitreeB1Z1RobotClient::get_twist_state_time_point gets the time point associated with the last twist state message
 *      received.
 * @return The desired time point.
 */
std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> UnitreeB1Z1RobotClient::get_twist_state_time_point() const
{
    return twist_state_time_point_;
}

/**
 * @brief UnitreeB1Z1RobotClient::get_arm_joint_state_time_point gets the time point associated with the last arm joint state message
 * @return The desired time point.
 */
std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> UnitreeB1Z1RobotClient::get_arm_joint_state_time_point() const
{
    return z1_arm_state_time_point_;
}


void UnitreeB1Z1RobotClient::set_walking_mode()
{
    _set_high_level_mode(static_cast<int>(HighLevelMode::TARGET_VELOCITY_WALKING));
}

void UnitreeB1Z1RobotClient::set_stand_mode()
{
    _set_high_level_mode(static_cast<int>(HighLevelMode::FORCED_STAND));
}

void UnitreeB1Z1RobotClient::_set_high_level_mode(const int &mode)
{
    if (mode_ == MODE::CONTROL && publisher_mode_switch_)
    {
        std_msgs::msg::Int32MultiArray mode_msg;
        // Set the mode value (e.g., 1 for FORCED_STAND, 2 for TARGET_VELOCITY_WALKING)
        mode_msg.data.push_back(mode);  // Walking mode
        publisher_mode_switch_->publish(mode_msg);
    }
}

void UnitreeB1Z1RobotClient::set_forced_stand_commands(const double &roll_angle,
                                                       const double &pitch_angle,
                                                       const double &yaw_angle,
                                                       const double &bodyheight)
{
    if (mode_ == MODE::CONTROL && publisher_stand_commands_)
    {
        std_msgs::msg::Float64MultiArray stand_msg;
        stand_msg.data.clear();
        stand_msg.data.push_back(roll_angle);   // Roll angle (radians)
        stand_msg.data.push_back(pitch_angle);  // Pitch angle (radians)
        stand_msg.data.push_back(yaw_angle);    // Yaw angle (radians)
        stand_msg.data.push_back(bodyheight);   // Body height (meters)
        // Publish the message
        publisher_stand_commands_->publish(stand_msg);
    }
}


UnitreeB1Z1RobotClient::~UnitreeB1Z1RobotClient()
{

}

}
