function in = sensor_env_module_get_activate()
%SENSOR_ENV_MODULE_GET_ACTIVATE Summary of this function goes here
%   Detailed explanation goes here

file_name_param = 'sensor_env_module_parameters';
[~,str_core] = get_multi_core_value();
fun_params = str2func([file_name_param, str_core]);

[activate_sensor] = fun_params();

in = activate_sensor;
end

