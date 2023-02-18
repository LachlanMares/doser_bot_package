#include <ros/ros.h>
#include <doser_bot_package/EnableMotorService.h>
#include <doser_bot_package/DisableMotorService.h>
#include <parameter_lookup.hpp>
#include <doser_bot_package/InterfaceConfig.h>


class TrackDynamicParameterChanges {
    private:
        // Node handles
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;
        
        // Services
        ros::ServiceClient _enable_motor_client;
        ros::ServiceClient _disable_motor_client;

        // Timer
        ros::Timer timer;

        std::string _enable_motor_service;
        std::string _disable_motor_service;

        ParameterLookUp pl;
        dynamic_settings_struct _settings_parameters;
        dynamic_settings_values_struct _prev_settings;

    public:
        TrackDynamicParameterChanges(): _nh(), _nh_priv(), pl(){
            // Private Parameters
            _nh_priv.param<std::string>("enable_motor_service_name", _enable_motor_service, "motor_enable_service");
            _nh_priv.param<std::string>("disable_motor_service_name", _disable_motor_service, "motor_disable_service");

            _enable_motor_client = _nh.serviceClient<doser_bot_package::EnableMotorService>(_enable_motor_service); 
            _disable_motor_client = _nh.serviceClient<doser_bot_package::DisableMotorService>(_disable_motor_service); 

            // Variables
            _settings_parameters = pl.getSettingsParameters();
            _prev_settings = _settings_parameters.values;

            // Timers
            timer = _nh.createTimer(ros::Duration(0.2), &TrackDynamicParameterChanges::timerCallback, this);
        };

        void timerCallback(const ros::TimerEvent &) {        
            dynamic_settings_values_struct new_settings = pl.getSettingsValues(&_settings_parameters);

            for (int i=0; i<10; i++) {
                if (new_settings.enable[i] != _prev_settings.enable[i]) {
                    if (new_settings.enable[i]) {
                        doser_bot_package::EnableMotorService motor_enable;
                        motor_enable.request.motor = i;

                        if (!_enable_motor_client.call(motor_enable)) {
                            ROS_ERROR("Failed to call service enable motor");
                        }   

                    } else {
                        doser_bot_package::DisableMotorService motor_disable;
                        motor_disable.request.motor = i;

                        if (!_disable_motor_client.call(motor_disable)) {
                            ROS_ERROR("Failed to call service disable motor");
                        }   
                    }
                }
            }

            _prev_settings = new_settings;
        }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "track_parameter_changes");
   
    TrackDynamicParameterChanges track_dynamic_parameter_changes;

    ros::spin();

    return 0;
}