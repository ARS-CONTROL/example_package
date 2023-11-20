#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {

    // Initialize a ROS2 Node
    rclcpp::init(argc, argv);

    // Allow Undeclared Parameters
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);

    // Create the ROS2 Node
    auto node = std::make_shared<rclcpp::Node>("global_parameter_server", options);

    // Spin the Node
    rclcpp::spin(node);

    // Shutdown the Node
    rclcpp::shutdown();
    return 0;

}