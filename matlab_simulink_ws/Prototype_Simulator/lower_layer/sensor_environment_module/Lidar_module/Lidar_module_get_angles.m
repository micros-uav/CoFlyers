function angles = Lidar_module_get_angles(parameters_Lidar)
%LIDAR_MODULE_GET_ANGLES 此处显示有关此函数的摘要
%   此处显示详细说明
[angle_min, angle_max, angle_increment, ~, ~,...
    ~, ~] = Lidar_module_parameters_deal(parameters_Lidar);
angles = angle_min:angle_increment:angle_max;
end

