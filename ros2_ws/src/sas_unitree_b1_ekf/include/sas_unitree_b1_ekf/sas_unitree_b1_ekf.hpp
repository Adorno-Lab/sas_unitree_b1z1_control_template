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

#pragma once
#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <dqrobotics/DQ.h>
#include <sas_core/sas_clock.hpp>
#include <sas_tools/sas_legged_robot_driver_client.hpp>

using namespace DQ_robotics;

namespace sas
{

struct UnitreeB1EKFConfiguration
{
    std::string b1_topic_prefix;                          ///< e.g. "sas_b1/b1_1"
    double thread_sampling_time_sec;
    std::string world_frame;                              ///< tf frame of Vicon, e.g. "world"
    std::string robot_vicon_marker_rear;
    std::string robot_vicon_marker_front;
    double marker_timeout_sec;                            ///< A marker without new data for this long is lost
    double max_marker_tilt_rad;                           ///< Markers more tilted than this are rejected
    int initialization_samples;                           ///< Vicon samples averaged at the start
    std::vector<double> rear_marker_to_trunk_translation; ///< {x, y, z} in the frame of the rear marker
    std::vector<double> process_noise;                    ///< diagonal of R {x, y, phi}, per iteration
    std::vector<double> measurement_noise;                ///< diagonal of Q {x, y, phi}
};

/**
 * @brief Planar EKF for the B1. State {x, y, phi} of the trunk. The prediction uses the measured twist
 *        (get/twist of the driver) and the correction uses the Vicon markers (tf). The height comes from Vicon.
 *        Publishes the pose of the trunk on <b1_topic_prefix>/get/ekf/robot_pose.
 *
 *        The notation (SIGMA, R, Q, G, K) follows the EKF of Section 3.3 of [1].
 *
 *        [1] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics," MIT Press, 2005.
 */
class UnitreeB1EKF
{
private:
    struct Marker
    {
        std::string name;
        DQ pose{1};
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
        double last_update_time_sec{-1.0}; ///< Elapsed time of the clock_ when the last new sample arrived
        bool valid{false};
        bool new_sample{false};
    };

    std::shared_ptr<rclcpp::Node> node_;
    UnitreeB1EKFConfiguration configuration_;
    std::atomic_bool* break_loops_;
    Clock clock_;

    std::shared_ptr<LeggedRobotDriverClient> b1_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_robot_pose_;

    Marker rear_;
    Marker front_;
    DQ x_rear_to_front_{1};
    DQ x_rear_to_trunk_{1};
    double height_{0};

    Eigen::Vector3d state_;
    Eigen::Matrix3d SIGMA_;
    Eigen::Matrix3d R_;
    Eigen::Matrix3d Q_;

    bool _should_shutdown() const;
    void _spin_and_sleep();
    void _update_marker(Marker& marker);
    void _update_markers();
    bool _get_measurement(DQ& x_trunk);
    void _prediction_step(const double& dt);
    void _correction_step(const DQ& x_trunk);
    DQ _get_estimated_pose() const;
    void _publish_estimated_pose();
    void _initialize();

    static DQ _transform_to_dq(const geometry_msgs::msg::TransformStamped& msg);
    static double _get_yaw(const DQ& x);
    static double _wrap_angle(const double& angle);
    static DQ _running_mean(const DQ& mean, const DQ& x, const int& n);

public:
    UnitreeB1EKF() = delete;
    UnitreeB1EKF(const UnitreeB1EKF&) = delete;

    UnitreeB1EKF(const std::shared_ptr<rclcpp::Node>& node,
                 const UnitreeB1EKFConfiguration& configuration,
                 std::atomic_bool* break_loops);

    /**
     * @brief Waits for the driver and both markers, initializes the filter, and runs it until break_loops is set.
     */
    void loop();
};

}
