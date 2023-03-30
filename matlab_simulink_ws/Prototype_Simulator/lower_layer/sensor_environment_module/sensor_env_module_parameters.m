function parameters_sensor_env = sensor_env_module_parameters(flag_sensor,p_bp)
%SENSOR_ENV_MODULE_PARAMETERS 此处显示有关此函数的摘要
%   此处显示详细说明

switch flag_sensor
    case 0
        parameters_sensor_env = [flag_sensor;Lidar_module_parameters(p_bp)];
    otherwise
        parameters_sensor_env = [];
end

end

