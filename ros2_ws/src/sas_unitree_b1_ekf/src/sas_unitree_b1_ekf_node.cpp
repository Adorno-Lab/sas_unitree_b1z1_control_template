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

#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include "sas_unitree_b1_ekf/sas_unitree_b1_ekf.hpp"

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>

static std::atomic_bool kill_this_process(false);

void sig_int_handler(int);

void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");

    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_unitree_b1_ekf");

    try
    {
        sas::UnitreeB1EKFConfiguration configuration;
        sas::get_ros_parameter(node, "b1_topic_prefix", configuration.b1_topic_prefix);
        sas::get_ros_parameter(node, "thread_sampling_time_sec", configuration.thread_sampling_time_sec);
        sas::get_ros_parameter(node, "world_frame", configuration.world_frame);
        sas::get_ros_parameter(node, "robot_vicon_marker_rear", configuration.robot_vicon_marker_rear);
        sas::get_ros_parameter(node, "robot_vicon_marker_front", configuration.robot_vicon_marker_front);
        sas::get_ros_parameter(node, "marker_timeout_sec", configuration.marker_timeout_sec);
        sas::get_ros_parameter(node, "max_marker_tilt_rad", configuration.max_marker_tilt_rad);
        sas::get_ros_parameter(node, "initialization_samples", configuration.initialization_samples);
        sas::get_ros_parameter(node, "rear_marker_to_trunk_translation", configuration.rear_marker_to_trunk_translation);
        sas::get_ros_parameter(node, "process_noise", configuration.process_noise);
        sas::get_ros_parameter(node, "measurement_noise", configuration.measurement_noise);
        RCLCPP_INFO_STREAM(node->get_logger(), "::Parameters OK.");

        sas::UnitreeB1EKF ekf(node, configuration, &kill_this_process);
        ekf.loop();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "::Exception::" << e.what());
    }

    rclcpp::shutdown();
    return 0;
}
