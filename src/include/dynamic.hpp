/*
Author:
    Lachlan Mares, lachlan.mares@adelaide.edu.au

License:
    GPL-3.0

Description:

*/

#ifndef __D_H_
#define __D_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <doser_bot_package/InterfaceConfig.h>

#include <string>
#include <memory>


class dynamic_class
{
    private:
        // ROS declaration
        ros::NodeHandle private_nh_;

        // Dynamic reconfigure handle
        std::shared_ptr<dynamic_reconfigure::Server<doser_bot_package_dynamic_reconfigure::InterfaceConfig>> server_;

    public:
        // Constructor and destructor
        dynamic_class();
        ~dynamic_class();

        // Public function
        void start();
};

#endif