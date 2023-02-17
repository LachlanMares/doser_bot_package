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

        // Variables
        dynamic_multi_motor_struct _multi_motor_parameters;
        dynamic_selection_struct _selection_parameters;
        dynamic_settings_struct _settings_parameters;

        // Publishers
        ros::Publisher motor_job_publisher;

        // Timers
        ros::Timer timer;

    public:
        HandleSelections(std::string motor_topic_str): _nh() {
            // Publishers
            motor_job_publisher = _nh.advertise<doser_bot_package::MultiMotorRotationsAtRpmJob>(motor_topic_str, 1);

            // Variables
            _multi_motor_parameters = pl.getMotorParameters();
            _selection_parameters = pl.getSelectionParameters();
            _settings_parameters = pl.getSettingsParameters();

            // Timers
            timer = _nh.createTimer(ros::Duration(0.1), &HandleSelections::timerCallback, this);
        };

    void timerCallback(const ros::TimerEvent &) {    
        dynamic_selection_values_struct selection_values = pl.getSelectionValues(&_selection_parameters);

        if(selection_values.make_it_so) {
            dynamic_multi_motor_values_struct motor_values = pl.getMotorValues(&_multi_motor_parameters);
            dynamic_settings_values_struct settings_values = pl.getSettingsValues(&_settings_parameters);

            doser_bot_package::MultiMotorRotationsAtRpmJob multi_motor_job;

            double contributions[10] = {selection_values.liquid_0,
                                        selection_values.liquid_1,
                                        selection_values.liquid_2,
                                        selection_values.liquid_3,
                                        selection_values.liquid_4,
                                        selection_values.liquid_5,
                                        selection_values.liquid_6,
                                        selection_values.liquid_7,
                                        selection_values.liquid_8,
                                        selection_values.liquid_9
                                        };

            for (int i=0; i<10; i++) {
                multi_motor_job.motor[i].motor_id = motor_values.motor[i].motor_id;
                multi_motor_job.motor[i].direction = motor_values.motor[i].direction;
                multi_motor_job.motor[i].use_ramping = motor_values.motor[i].use_ramping;
                multi_motor_job.motor[i].job_id = selection_values.job_id;
                multi_motor_job.motor[i].rpm = motor_values.motor[i].rpm;
                multi_motor_job.motor[i].ramping_steps = motor_values.motor[i].ramping_steps;
                multi_motor_job.motor[i].rotations = settings_values.motor_rotations_per_drink * contributions[i];
            }

            motor_job_publisher.publish(multi_motor_job);

            pl.resetDrinkSelection(&_selection_parameters);
        }
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "handle_selections_node");
    
    std::string motor_topic_str = "/multi_motor_rotations_at_rpm";

    if (argc > 1) {
        motor_topic_str = argv[1];
    }

  HandleSelections handle_selections(motor_topic_str);

  ros::spin();

  return 0;
}