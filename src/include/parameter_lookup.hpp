/*
Author:
    Lachlan Mares, lachlan.mares@adelaide.edu.au

License:
    GPL-3.0

Description:

*/

#include <ros/ros.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <sstream>

typedef struct dmis {
    int motor_id_index;
    int direction_index;
    int use_ramping_index;
    int ramping_steps_index;
    int rpm_index;
} dynamic_motor_index_struct;

typedef struct dmss {
    std::string motor_id_str;
    std::string direction_str;
    std::string use_ramping_str;
    std::string ramping_steps_str;
    std::string rpm_str;
} dynamic_motor_strings_struct;

typedef struct dmvs {
    bool direction;
    bool use_ramping;
    int motor_id;
    int ramping_steps;
    double rpm;
} dynamic_motor_values_struct;

typedef struct dms {
    dynamic_motor_index_struct indexes;
    dynamic_motor_values_struct values;
    dynamic_motor_strings_struct strings;
} dynamic_motor_struct;

typedef struct dmms {
    dynamic_motor_struct motor[10];
} dynamic_multi_motor_struct;

typedef struct dmmvs {
    dynamic_motor_values_struct motor[10];
} dynamic_multi_motor_values_struct;

typedef struct dseis {
    int motor_rotations_per_drink_index;
    int enable_index[10];
} dynamic_settings_index_struct;

typedef struct dsess {
    std::string motor_rotations_per_drink_str;
    std::string enable_str[10];
} dynamic_settings_string_struct;

typedef struct dsevs {
    int motor_rotations_per_drink;
    bool enable[10];
} dynamic_settings_values_struct;

typedef struct dess {
    dynamic_settings_index_struct indexes;
    dynamic_settings_string_struct strings;
    dynamic_settings_values_struct values;
} dynamic_settings_struct;

typedef struct dsis {
    int drink_mode_index;
    int job_id_index;
    int liquid_0_index;
    int liquid_1_index;
    int liquid_2_index;
    int liquid_3_index;
    int liquid_4_index;
    int liquid_5_index;
    int liquid_6_index;
    int liquid_7_index;
    int liquid_8_index;
    int liquid_9_index;
    int make_it_so_index;
} dynamic_selection_index_struct;

typedef struct dsss {
    std::string drink_mode_str;
    std::string job_id_str;
    std::string liquid_0_str;
    std::string liquid_1_str;
    std::string liquid_2_str;
    std::string liquid_3_str;
    std::string liquid_4_str;
    std::string liquid_5_str;
    std::string liquid_6_str;
    std::string liquid_7_str;
    std::string liquid_8_str;
    std::string liquid_9_str;
    std::string make_it_so_str;
} dynamic_selection_strings_struct;

typedef struct dsvs {
    int drink_mode;
    int job_id;
    double liquid_0;
    double liquid_1;
    double liquid_2;
    double liquid_3;
    double liquid_4;
    double liquid_5;
    double liquid_6;
    double liquid_7;
    double liquid_8;
    double liquid_9;
    bool make_it_so;
} dynamic_selection_values_struct;

typedef struct dss {
    dynamic_selection_index_struct indexes;
    dynamic_selection_values_struct values;
    dynamic_selection_strings_struct strings;
} dynamic_selection_struct;

class ParameterLookUp
{
    private:
        // Node handles
        ros::NodeHandle _nh;
        std::string _dynamic_reconfigure_node_str;

        dynamic_multi_motor_struct _motor_parameters;
        dynamic_selection_struct _selection_parameters;
        dynamic_settings_struct _settings_parameters;

    public:
        ParameterLookUp(): _nh() {
            _nh.param<std::string>("dynamic_reconfigure_node_name", _dynamic_reconfigure_node_str, "dynamic_node");
            _dynamic_reconfigure_node_str = "/" + _dynamic_reconfigure_node_str + "/set_parameters";
            _motor_parameters = getMotorParameters();
            _selection_parameters = getSelectionParameters();
            _settings_parameters = getSettingsParameters();
        }

        int getDynamicParameterIndexBool(std::string param_name_str) {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);

            int param_index = -1;

            for (int i=0; i<srv_resp.config.bools.size(); i++){
                if (srv_resp.config.bools[i].name.compare(param_name_str) == 0){
                    param_index = i;
                } 
            }
            return param_index;
        }

        int getDynamicParameterIndexInt(std::string param_name_str) {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);

            int param_index = -1;

            for (int i=0; i<srv_resp.config.ints.size(); i++){
                if (srv_resp.config.ints[i].name.compare(param_name_str) == 0){
                    param_index = i;
                } 
            }
            return param_index;
        }

        int getDynamicParameterIndexDouble(std::string param_name_str) {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);

            int param_index = -1;

            for (int i=0; i<srv_resp.config.ints.size(); i++){
                if (srv_resp.config.doubles[i].name.compare(param_name_str) == 0){
                    param_index = i;
                } 
            }
            return param_index;
        }

        bool getDynamicParameterValueBool(int param_index) {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);
            return srv_resp.config.bools[param_index].value;
        }

        void setDynamicParameterValueBool(std::string param_name_str, bool parameter_value) {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            dynamic_reconfigure::BoolParameter temp_parameter;
            dynamic_reconfigure::Config conf;

            temp_parameter.name = param_name_str;
            temp_parameter.value = parameter_value;
            conf.bools.push_back(temp_parameter);
            srv_req.config = conf;

            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);
        }

        int getDynamicParameterValueInt(int param_index) {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);

            return srv_resp.config.ints[param_index].value;
        }

        void setDynamicParameterValueInt(std::string param_name_str, int parameter_value) {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            dynamic_reconfigure::IntParameter temp_parameter;
            dynamic_reconfigure::Config conf;

            temp_parameter.name = param_name_str;
            temp_parameter.value = parameter_value;
            conf.ints.push_back(temp_parameter);
            srv_req.config = conf;

            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);
        }

        double getDynamicParameterValueDouble(int param_index) {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);

            return srv_resp.config.doubles[param_index].value;
        }

        void setDynamicParameterValueDouble(std::string param_name_str, double parameter_value) {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            dynamic_reconfigure::DoubleParameter temp_parameter;
            dynamic_reconfigure::Config conf;

            temp_parameter.name = param_name_str;
            temp_parameter.value = parameter_value;
            conf.doubles.push_back(temp_parameter);
            srv_req.config = conf;

            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);
        }

        /***********************************************************************
        Motors
        ***********************************************************************/

        dynamic_multi_motor_struct getMotorParameters() {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);
            
            dynamic_multi_motor_struct return_struct;

            for (int h=0; h<10; h++){
                std::ostringstream motor_id, motor_dir, use_ramping, ramping_steps, rpm;


                motor_id << "motor_" << h << "_id";
                motor_dir << "motor_" << h << "_direction";
                use_ramping << "motor_" << h << "_use_ramping";
                ramping_steps << "motor_" << h << "_ramping_steps";
                rpm << "motor_" << h << "_rpm";

                return_struct.motor[h].strings.motor_id_str = motor_id.str();
                return_struct.motor[h].strings.direction_str = motor_dir.str();
                return_struct.motor[h].strings.use_ramping_str = use_ramping.str();
                return_struct.motor[h].strings.ramping_steps_str = ramping_steps.str();
                return_struct.motor[h].strings.rpm_str = rpm.str();

                for (int i=0; i<srv_resp.config.bools.size(); i++){
                    if (srv_resp.config.bools[i].name.compare(return_struct.motor[h].strings.direction_str) == 0) {
                        return_struct.motor[h].indexes.direction_index = i;
                    } else if (srv_resp.config.bools[i].name.compare(return_struct.motor[h].strings.use_ramping_str) == 0) {
                        return_struct.motor[h].indexes.use_ramping_index = i;
                    }  
                }

                for (int j=0; j<srv_resp.config.ints.size(); j++){
                    if (srv_resp.config.ints[j].name.compare(return_struct.motor[h].strings.ramping_steps_str) == 0){
                        return_struct.motor[h].indexes.ramping_steps_index = j;
                    } else if (srv_resp.config.ints[j].name.compare(return_struct.motor[h].strings.motor_id_str) == 0){
                        return_struct.motor[h].indexes.motor_id_index = j;
                    }
                }

                for (int k=0; k<srv_resp.config.doubles.size(); k++){
                    if (srv_resp.config.doubles[k].name.compare(return_struct.motor[h].strings.rpm_str) == 0){
                        return_struct.motor[h].indexes.rpm_index = k;
                    } 
                }

                return_struct.motor[h].values.motor_id = srv_resp.config.bools[return_struct.motor[h].indexes.motor_id_index].value;
                return_struct.motor[h].values.direction = srv_resp.config.bools[return_struct.motor[h].indexes.direction_index].value;
                return_struct.motor[h].values.use_ramping = srv_resp.config.bools[return_struct.motor[h].indexes.use_ramping_index].value;
                return_struct.motor[h].values.ramping_steps = srv_resp.config.ints[return_struct.motor[h].indexes.ramping_steps_index].value;
                return_struct.motor[h].values.rpm = srv_resp.config.doubles[return_struct.motor[h].indexes.rpm_index].value;
            }
            return return_struct;
        }

        dynamic_multi_motor_values_struct getMotorValues() {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);
            
            dynamic_multi_motor_values_struct return_struct;

            for (int h=0; h<10; h++){
                return_struct.motor[h].motor_id = srv_resp.config.bools[_motor_parameters.motor[h].indexes.motor_id_index].value;
                return_struct.motor[h].direction = srv_resp.config.bools[_motor_parameters.motor[h].indexes.direction_index].value;
                return_struct.motor[h].use_ramping = srv_resp.config.bools[_motor_parameters.motor[h].indexes.use_ramping_index].value;
                return_struct.motor[h].ramping_steps = srv_resp.config.ints[_motor_parameters.motor[h].indexes.ramping_steps_index].value;
                return_struct.motor[h].rpm = srv_resp.config.doubles[_motor_parameters.motor[h].indexes.rpm_index].value;
            }
            return return_struct;
        }

        dynamic_multi_motor_values_struct getMotorValues(dynamic_multi_motor_struct* motor_parameters) {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);

            dynamic_multi_motor_values_struct return_struct;

            for (int h=0; h<10; h++){
                return_struct.motor[h].motor_id = srv_resp.config.bools[motor_parameters->motor[h].indexes.motor_id_index].value;
                return_struct.motor[h].direction = srv_resp.config.bools[motor_parameters->motor[h].indexes.direction_index].value;
                return_struct.motor[h].use_ramping = srv_resp.config.bools[motor_parameters->motor[h].indexes.use_ramping_index].value;
                return_struct.motor[h].ramping_steps = srv_resp.config.ints[motor_parameters->motor[h].indexes.ramping_steps_index].value;
                return_struct.motor[h].rpm = srv_resp.config.doubles[motor_parameters->motor[h].indexes.rpm_index].value;
            }
            return return_struct;
        }

        /***********************************************************************
        Drink Selections
        ***********************************************************************/

        dynamic_selection_struct getSelectionParameters() {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);
            
            dynamic_selection_struct return_struct;

            return_struct.strings.drink_mode_str = "drink_selection_mode";
            return_struct.strings.job_id_str = "job_id";
            return_struct.strings.liquid_0_str = "liquid_0";
            return_struct.strings.liquid_1_str = "liquid_1";
            return_struct.strings.liquid_2_str = "liquid_2";
            return_struct.strings.liquid_3_str = "liquid_3";
            return_struct.strings.liquid_4_str = "liquid_4";
            return_struct.strings.liquid_5_str = "liquid_5";
            return_struct.strings.liquid_6_str = "liquid_6";
            return_struct.strings.liquid_7_str = "liquid_7";
            return_struct.strings.liquid_8_str = "liquid_8";
            return_struct.strings.liquid_9_str = "liquid_9";
            return_struct.strings.make_it_so_str = "make_it_so";

            for (int i=0; i<srv_resp.config.bools.size(); i++){
                if (srv_resp.config.bools[i].name.compare(return_struct.strings.make_it_so_str) == 0){
                    return_struct.indexes.make_it_so_index = i;
                } 
            }

            for (int j=0; j<srv_resp.config.ints.size(); j++){
                if (srv_resp.config.ints[j].name.compare(return_struct.strings.drink_mode_str) == 0){
                    return_struct.indexes.drink_mode_index = j;
                } else if (srv_resp.config.ints[j].name.compare(return_struct.strings.job_id_str) == 0){
                    return_struct.indexes.job_id_index = j;
                }
            }

            for (int k=0; k<srv_resp.config.doubles.size(); k++){
                if (srv_resp.config.doubles[k].name.compare(return_struct.strings.liquid_0_str) == 0){
                    return_struct.indexes.liquid_0_index = k;
                } else if (srv_resp.config.doubles[k].name.compare(return_struct.strings.liquid_1_str) == 0){
                    return_struct.indexes.liquid_1_index = k;
                } else if (srv_resp.config.doubles[k].name.compare(return_struct.strings.liquid_2_str) == 0){
                    return_struct.indexes.liquid_2_index = k;
                } else if (srv_resp.config.doubles[k].name.compare(return_struct.strings.liquid_3_str) == 0){
                    return_struct.indexes.liquid_3_index = k;
                } else if (srv_resp.config.doubles[k].name.compare(return_struct.strings.liquid_4_str) == 0){
                    return_struct.indexes.liquid_4_index = k;
                } else if (srv_resp.config.doubles[k].name.compare(return_struct.strings.liquid_5_str) == 0){
                    return_struct.indexes.liquid_5_index = k;
                } else if (srv_resp.config.doubles[k].name.compare(return_struct.strings.liquid_6_str) == 0){
                    return_struct.indexes.liquid_6_index = k;
                } else if (srv_resp.config.doubles[k].name.compare(return_struct.strings.liquid_7_str) == 0){
                    return_struct.indexes.liquid_7_index = k;
                } else if (srv_resp.config.doubles[k].name.compare(return_struct.strings.liquid_8_str) == 0){
                    return_struct.indexes.liquid_8_index = k;
                } else if (srv_resp.config.doubles[k].name.compare(return_struct.strings.liquid_9_str) == 0){
                    return_struct.indexes.liquid_9_index = k;
                } 
            }

            return_struct.values.drink_mode = srv_resp.config.ints[return_struct.indexes.drink_mode_index].value;
            return_struct.values.job_id = srv_resp.config.ints[return_struct.indexes.job_id_index].value;
            return_struct.values.liquid_0 = srv_resp.config.doubles[return_struct.indexes.liquid_0_index].value;
            return_struct.values.liquid_1 = srv_resp.config.doubles[return_struct.indexes.liquid_1_index].value;
            return_struct.values.liquid_2 = srv_resp.config.doubles[return_struct.indexes.liquid_2_index].value;
            return_struct.values.liquid_3 = srv_resp.config.doubles[return_struct.indexes.liquid_3_index].value;
            return_struct.values.liquid_4 = srv_resp.config.doubles[return_struct.indexes.liquid_4_index].value;
            return_struct.values.liquid_5 = srv_resp.config.doubles[return_struct.indexes.liquid_5_index].value;
            return_struct.values.liquid_6 = srv_resp.config.doubles[return_struct.indexes.liquid_6_index].value;
            return_struct.values.liquid_7 = srv_resp.config.doubles[return_struct.indexes.liquid_7_index].value;
            return_struct.values.liquid_8 = srv_resp.config.doubles[return_struct.indexes.liquid_8_index].value;
            return_struct.values.liquid_9 = srv_resp.config.doubles[return_struct.indexes.liquid_9_index].value;
            return_struct.values.make_it_so = srv_resp.config.bools[return_struct.indexes.make_it_so_index].value;

            return return_struct;
        }

        dynamic_selection_values_struct getSelectionValues(dynamic_selection_struct* _selection_parameters) {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);
            
            dynamic_selection_values_struct return_struct;

            return_struct.drink_mode = srv_resp.config.ints[_selection_parameters->indexes.drink_mode_index].value;
            return_struct.job_id = srv_resp.config.ints[_selection_parameters->indexes.job_id_index].value;
            return_struct.liquid_0 = srv_resp.config.doubles[_selection_parameters->indexes.liquid_0_index].value;
            return_struct.liquid_1 = srv_resp.config.doubles[_selection_parameters->indexes.liquid_1_index].value;
            return_struct.liquid_2 = srv_resp.config.doubles[_selection_parameters->indexes.liquid_2_index].value;
            return_struct.liquid_3 = srv_resp.config.doubles[_selection_parameters->indexes.liquid_3_index].value;
            return_struct.liquid_4 = srv_resp.config.doubles[_selection_parameters->indexes.liquid_4_index].value;
            return_struct.liquid_5 = srv_resp.config.doubles[_selection_parameters->indexes.liquid_5_index].value;
            return_struct.liquid_6 = srv_resp.config.doubles[_selection_parameters->indexes.liquid_6_index].value;
            return_struct.liquid_7 = srv_resp.config.doubles[_selection_parameters->indexes.liquid_7_index].value;
            return_struct.liquid_8 = srv_resp.config.doubles[_selection_parameters->indexes.liquid_8_index].value;
            return_struct.liquid_9 = srv_resp.config.doubles[_selection_parameters->indexes.liquid_9_index].value;
            return_struct.make_it_so = srv_resp.config.bools[_selection_parameters->indexes.make_it_so_index].value;

            return return_struct;
        }

        void resetDrinkSelection(dynamic_selection_struct* _selection_parameters) {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            dynamic_reconfigure::BoolParameter temp_bool_parameter;
            dynamic_reconfigure::IntParameter temp_int_parameter;
            dynamic_reconfigure::DoubleParameter temp_double_parameter;
            dynamic_reconfigure::Config conf;

            temp_int_parameter.name = _selection_parameters->strings.drink_mode_str;
            temp_int_parameter.value = 0;
            conf.ints.push_back(temp_int_parameter);

            temp_int_parameter.name = _selection_parameters->strings.job_id_str;
            temp_int_parameter.value = _selection_parameters->values.job_id + 1;
            conf.ints.push_back(temp_int_parameter);

            temp_double_parameter.name = _selection_parameters->strings.liquid_0_str;
            temp_double_parameter.value = 0.0;
            conf.doubles.push_back(temp_double_parameter);

            temp_double_parameter.name = _selection_parameters->strings.liquid_1_str;
            temp_double_parameter.value = 0.0;
            conf.doubles.push_back(temp_double_parameter);

            temp_double_parameter.name = _selection_parameters->strings.liquid_2_str;
            temp_double_parameter.value = 0.0;
            conf.doubles.push_back(temp_double_parameter);

            temp_double_parameter.name = _selection_parameters->strings.liquid_3_str;
            temp_double_parameter.value = 0.0;
            conf.doubles.push_back(temp_double_parameter);
            
            temp_double_parameter.name = _selection_parameters->strings.liquid_4_str;
            temp_double_parameter.value = 0.0;
            conf.doubles.push_back(temp_double_parameter);
            
            temp_double_parameter.name = _selection_parameters->strings.liquid_5_str;
            temp_double_parameter.value = 0.0;
            conf.doubles.push_back(temp_double_parameter);
            
            temp_double_parameter.name = _selection_parameters->strings.liquid_6_str;
            temp_double_parameter.value = 0.0;
            conf.doubles.push_back(temp_double_parameter);

            temp_double_parameter.name = _selection_parameters->strings.liquid_7_str;
            temp_double_parameter.value = 0.0;
            conf.doubles.push_back(temp_double_parameter);
            
            temp_double_parameter.name = _selection_parameters->strings.liquid_8_str;
            temp_double_parameter.value = 0.0;
            conf.doubles.push_back(temp_double_parameter);
            
            temp_double_parameter.name = _selection_parameters->strings.liquid_9_str;
            temp_double_parameter.value = 0.0;
            conf.doubles.push_back(temp_double_parameter);

            temp_bool_parameter.name = _selection_parameters->strings.make_it_so_str;
            temp_bool_parameter.value = false;
            conf.bools.push_back(temp_bool_parameter);

            srv_req.config = conf;

            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);
        }

        /***********************************************************************
        Settings
        ***********************************************************************/

         dynamic_settings_struct getSettingsParameters() {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);

            dynamic_settings_struct return_struct;

            return_struct.strings.motor_rotations_per_drink_str = "motor_rotations_per_drink";
            
            for (int j=0; j<srv_resp.config.ints.size(); j++) {
                if (srv_resp.config.ints[j].name.compare(return_struct.strings.motor_rotations_per_drink_str) == 0) {
                    return_struct.indexes.motor_rotations_per_drink_index = j;
                }
            }

            return_struct.values.motor_rotations_per_drink = srv_resp.config.ints[return_struct.indexes.motor_rotations_per_drink_index].value;

            for (int i=0; i<10; i++) {
                std::ostringstream motor_en;
                motor_en << "motor_" << i << "_enable";
                return_struct.strings.enable_str[i] = motor_en.str();

                for (int j=0; j<srv_resp.config.bools.size(); j++){
                    if (srv_resp.config.bools[j].name.compare(return_struct.strings.enable_str[i]) == 0) {
                        return_struct.indexes.enable_index[i] = j;
                    }
                }

                return_struct.values.enable[i] = srv_resp.config.bools[return_struct.indexes.enable_index[i]].value;
            }

            return return_struct;
        }

        dynamic_settings_values_struct getSettingsValues(dynamic_settings_struct* _settings_parameters) {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);

            dynamic_settings_values_struct return_struct;

            return_struct.motor_rotations_per_drink = srv_resp.config.ints[_settings_parameters->indexes.motor_rotations_per_drink_index].value;

            for (int i=0; i<10; i++) {
                return_struct.enable[i] = srv_resp.config.bools[_settings_parameters->indexes.enable_index[i]].value;
            }

            return return_struct;
        }

        void setSettingsValues(dynamic_settings_struct* _settings_parameters, int motor_rotations_per_drink) {
            dynamic_reconfigure::ReconfigureResponse srv_resp;
            dynamic_reconfigure::ReconfigureRequest srv_req;
            dynamic_reconfigure::IntParameter temp_int_parameter;
            dynamic_reconfigure::Config conf;

            temp_int_parameter.name = _settings_parameters->strings.motor_rotations_per_drink_str;
            temp_int_parameter.value = motor_rotations_per_drink;
            conf.ints.push_back(temp_int_parameter);

            srv_req.config = conf;

            ros::service::call(_dynamic_reconfigure_node_str, srv_req, srv_resp);
        }

};

