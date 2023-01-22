/*
Author:
    Lachlan Mares, lachlan.mares@adelaide.edu.au

License:
    GPL-3.0

Description:

*/

#include "dynamic.hpp"

dynamic_class::dynamic_class() : 
    private_nh_("~"), 
    server_(std::make_shared<dynamic_reconfigure::Server<doser_bot_package_dynamic_reconfigure::InterfaceConfig>> (private_nh_)){}

dynamic_class::~dynamic_class() {}

void dynamic_class::start()
{
    // Set callback function for dynamic reconfigure (using lambda)
    server_->setCallback(
        [this](doser_bot_package_dynamic_reconfigure::InterfaceConfig & config, uint32_t level){

        });

    // ROS spin (wait for server to enter callback)
    ros::spin();
}

const std::string RosNodeName = "dynamic_node";

int main(int argc, char ** argv)
{
    // ROS initialization
    ros::init(argc, argv, RosNodeName);

    // Instantiate dynamic_class
    dynamic_class node;

    // Start node
    node.start();

    return 0;
}