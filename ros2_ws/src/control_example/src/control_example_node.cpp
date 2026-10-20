#include <rclcpp/rclcpp.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <dqrobotics/utils/DQ_Math.h>
#include "control_example/control_example.hpp"

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
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    // Keep our SIGINT handler: the controller stops the robot before leaving.
    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("control_example_node");

    try
    {

        sas::ControlExampleConfiguration configuration;
        sas::get_ros_parameter(node,"B1_topic_prefix",configuration.B1_topic_prefix);
        sas::get_ros_parameter(node,"Z1_topic_prefix",configuration.Z1_topic_prefix);
        sas::get_ros_parameter(node,"x_prefix",configuration.x_prefix);
        sas::get_ros_parameter(node,"xd_prefix",configuration.xd_prefix);
        sas::get_ros_parameter(node,"trunk_to_arm_base_translation",configuration.trunk_to_arm_base_translation);
        sas::get_ros_parameter(node,"thread_sampling_time_sec",configuration.thread_sampling_time_sec);

        std::vector<double> configuration_limits_min;
        std::vector<double> configuration_limits_max;
        sas::get_ros_parameter(node,"configuration_limits_min_deg", configuration_limits_min);
        sas::get_ros_parameter(node,"configuration_limits_max_deg", configuration_limits_max);
        configuration.configuration_limits = {deg2rad(sas::std_vector_double_to_vectorxd(configuration_limits_min)),
                                              deg2rad(sas::std_vector_double_to_vectorxd(configuration_limits_max))};


        std::vector<double> config_vel_limits_min;
        std::vector<double> config_vel_limits_max;
        sas::get_ros_parameter(node,"config_vel_limits_min", config_vel_limits_min);
        sas::get_ros_parameter(node,"config_vel_limits_max", config_vel_limits_max);
        configuration.configuration_vel_limits = {sas::std_vector_double_to_vectorxd(config_vel_limits_min),
                                                  sas::std_vector_double_to_vectorxd(config_vel_limits_max)};

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Parameters OK.");
        auto controller = std::make_shared<sas::ControlExample>(node,configuration, &kill_this_process);
        controller->control_loop();

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
        std::cerr << std::string("::Exception::") << e.what();
    }

    rclcpp::shutdown();
    return 0;


}
