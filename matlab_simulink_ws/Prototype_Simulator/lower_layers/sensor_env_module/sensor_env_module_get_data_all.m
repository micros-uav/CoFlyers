function  sensor_data_s = sensor_env_module_get_data_all(states_ob, flag_motion_model,map_faces)
%SENSOR_ENV_MODULE_GET_DATA_ALL Summary of this function goes here
%   Detailed explanation goes here

number =size(states_ob,2);
if number < 1
    sensor_data_s = [];
    return
end
position3D_s = states_ob(1:3,:);
if strcmp(flag_motion_model,'quadcopter')
    % attitude_s = permute(states_ob(10:12,:),[2,3,1])*180/pi;
    attitude_s = states_ob(10:12,:)*180/pi;
else
    attitude_s = zeros(3,number);
end

flag_sensor       = 0;
switch flag_sensor
    case 0
%         angles = Lidar_module_get_angles(parameters_sensor);
        [range_s, psi_s, phi_s]  = lidar_module_get_ranges(position3D_s(1:3,1),attitude_s(1:3,1),map_faces);
        datas = [range_s;psi_s;phi_s];
        sensor_data_s = zeros([size(datas),number]);
        sensor_data_s(:,:,1) = datas;
        for id = 2:number
            [range_s, psi_s, phi_s]  = lidar_module_get_ranges(position3D_s(1:3,id),attitude_s(1:3,id),map_faces);
            datas = [range_s;psi_s;phi_s];
            sensor_data_s(:,:,id) = datas;
        end
    otherwise
        sensor_data_s = [];
end

end

