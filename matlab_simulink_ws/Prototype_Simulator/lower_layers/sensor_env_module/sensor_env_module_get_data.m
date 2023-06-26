function datas = sensor_env_module_get_data(position3D, attitude,map_faces)
%SENSOR_ENV_MODULE_GET_DATA_FROM_MAP_FACES
%   

flag_sensor       = 0;
switch flag_sensor
    case 0
%         angles = Lidar_module_get_angles(parameters_sensor);
        [range_s, psi_s, phi_s]  = lidar_module_get_ranges(position3D,attitude,map_faces);
        datas = [range_s;psi_s;phi_s];
    otherwise
        datas = [];
end

end

