#include "example_package/example.h"

void UR_Stop_Signal_Handler (int sig) {

    ros::NodeHandle nh;
    
    ros::Publisher joint_group_vel_controller_publisher = nh.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);

    std_msgs::Float64MultiArray stop_msgs;
    std::vector<double> stop_vector;
    stop_vector.resize(6, 0.0);

    stop_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());
    stop_msgs.layout.dim[0].size = stop_vector.size();
    stop_msgs.layout.dim[0].stride = 1;
    stop_msgs.layout.dim[0].label = "velocity";

    stop_msgs.data.clear();
    stop_msgs.data.insert(stop_msgs.data.end(), stop_vector.begin(), stop_vector.end());

    joint_group_vel_controller_publisher.publish(stop_msgs);

    ros::shutdown();

}

void custom_Signal_Handler (int sig) {

    ros::NodeHandle nh;
    
    // DO THINGS ...

    ros::shutdown();

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "example_Node", ros::init_options::NoSigintHandler);

    // Sig-Int Function (CTRL+C)
    signal(SIGINT, UR_Stop_Signal_Handler);
    signal(SIGINT, custom_Signal_Handler);

    // ROS Initialisation Parameters
    ros::NodeHandle nh;
    ros::Rate ros_rate = 1000;

    // Class Initialisation Parameters
    std::string example_string = "example_data";
    std::vector<double> example_vector = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    
    ExamplePackage *ep = new ExamplePackage(nh, ros_rate, example_string, example_vector);
                     
    while (ros::ok())
    {

        ep -> spinner();

    }

    delete ep;

return 0;

}
