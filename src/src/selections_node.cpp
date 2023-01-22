#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <doser_bot_package/MultiMotorRotationsAtRpmJob.h>
#include <parameter_lookup.hpp>

using namespace cv;


class HandleSelections {
    private:
        // Node handles
        ros::NodeHandle _nh;

        // dynamic reconfigure
        ParameterLookUp pl;

        // Publishers
        ros::Publisher motor_job_publisher;

        // Timers
        ros::Timer timer;

    public:
        HandleSelections(std::string motor_topic_str): _nh() {
            // Publishers
            motor_job_publisher = _nh.advertise<doser_bot_package::MultiMotorRotationsAtRpmJob>(motor_topic_str, 1);

            // Timers
            timer = _nh.createTimer(ros::Duration(0.1), &HandleSelections::timerCallback, this);

        };

    void timerCallback(const ros::TimerEvent &) {    
        dynamic_selection_values_struct _selection_parameters = pl.getSelectionValues();
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "handle_selections_node");
    
    std::string motor_topic_str = "/multi_motor_rotations_at_rpm";

    if (argc > 1) {
        motor_topic_str = argv[1];
    }

  HandleSelections hangle_selections(motor_topic_str);

  ros::spin();

  return 0;
}