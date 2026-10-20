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
#include <atomic>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/UnitreeB1Z1MobileRobot.h>
#include <sas_core/sas_clock.hpp>
#include <sas_common/sas_object_client.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>
#include <sas_tools/sas_legged_robot_driver_client.hpp>


using namespace rclcpp;
using namespace DQ_robotics;

namespace sas
{
struct ControlExampleConfiguration
{
    std::string B1_topic_prefix;                       ///< e.g. "sas_b1/b1_1"
    std::string Z1_topic_prefix;                       ///< e.g. "sas_z1/z1_1"
    std::string x_prefix;                              ///< e.g. "sas_robot_driver_coppeliasim/object/x1"
    std::string xd_prefix;                             ///< e.g. "sas_robot_driver_coppeliasim/object/xd1"
    std::vector<double> trunk_to_arm_base_translation; ///< From the trunk to the joint1 of the Z1, {x, y, z}
    double thread_sampling_time_sec;
    std::tuple<VectorXd, VectorXd> configuration_limits;
    std::tuple<VectorXd, VectorXd> configuration_vel_limits;
};

/**
 * @brief Kinematic control example for the B1Z1.
 *   - B1: LeggedRobotDriverClient on B1_topic_prefix. The base is commanded with a twist in WALKING.
 *   - Z1: RobotDriverClient on Z1_topic_prefix (6 joints and the gripper).
 *   - Pose of the trunk: <B1_topic_prefix>/get/ekf/robot_pose (the EKF, or the scene in simulation).
 *   - Frames x and xd: ObjectClients (e.g. the sas_object_script of the CoppeliaSim scene). They are optional.
 */
class ControlExample
{

private:
    ControlExampleConfiguration configuration_;
    std::atomic_bool* st_break_loops_;
    std::shared_ptr<rclcpp::Node> node_;
    Clock clock_;

    std::shared_ptr<LeggedRobotDriverClient> b1_client_;
    std::shared_ptr<RobotDriverClient> z1_client_;
    std::unique_ptr<ObjectClient> x_client_;
    std::unique_ptr<ObjectClient> xd_client_;

    Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_robot_pose_;
    DQ robot_pose_{0};
    void _callback_robot_pose(const geometry_msgs::msg::PoseStamped& msg);

    std::shared_ptr<UnitreeB1Z1MobileRobot> robot_model_;

    bool _should_shutdown() const;
    void _spin_and_sleep();
    VectorXd _get_configuration(const VectorXd& q_arm) const;
    void _set_b1_mode(const LeggedRobotDriver::HIGH_LEVEL_MODE& mode);
    void _stop_robot();
public:
    ControlExample(const ControlExample&)=delete;
    ControlExample()=delete;
    ~ControlExample();

    ControlExample(std::shared_ptr<Node>& node,
                   const ControlExampleConfiguration &configuration,
                   std::atomic_bool* break_loops);


    void control_loop();
};



}
