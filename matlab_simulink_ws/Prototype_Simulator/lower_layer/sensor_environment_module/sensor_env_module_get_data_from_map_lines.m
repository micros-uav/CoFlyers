function datas = sensor_env_module_get_data_from_map_lines(position3D,yaw,map_lines,parameters_sensor_env)
%SENSOR_ENV_MODULE_GET_DATA_FROM_MAP_LINES 此处显示有关此函数的摘要
%   此处显示详细说明

flag_sensor       = parameters_sensor_env(1);
parameters_sensor = parameters_sensor_env(2:end);
switch flag_sensor
    case 0
        angles = Lidar_module_get_angles(parameters_sensor);
        ranges = Lidar_module_get_ranges(position3D(1:2),yaw,map_lines,parameters_sensor);
        datas = [angles;ranges];
    otherwise
        datas = 0;
end
end

