function [angle_min, angle_max, angle_increment, time_increment, scan_time,...
    range_min, range_max] = Lidar_module_parameters_deal(parameters_Lidar)
%LIDAR_MODULE_PARAMETERS_DEAL 此处显示有关此函数的摘要
%   此处显示详细说明

myCount = 1;
angle_min       = parameters_Lidar(myCount); myCount = myCount +1;
angle_max       = parameters_Lidar(myCount); myCount = myCount +1;
angle_increment = parameters_Lidar(myCount); myCount = myCount +1;
time_increment  = parameters_Lidar(myCount); myCount = myCount +1;
scan_time       = parameters_Lidar(myCount); myCount = myCount +1;
range_min       = parameters_Lidar(myCount); myCount = myCount +1;
range_max       = parameters_Lidar(myCount); myCount = myCount +1;
end

