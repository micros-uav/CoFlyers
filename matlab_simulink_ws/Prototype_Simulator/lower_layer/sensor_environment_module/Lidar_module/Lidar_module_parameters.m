function parameters_Lidar = Lidar_module_parameters(p_bp)
%LIDAR_MODULE_PARAMETERS 此处显示有关此函数的摘要
%   此处显示详细说明

angle_min = -pi;
angle_max = pi;
angle_increment = 2*pi/359;
time_increment = 1e-7;
scan_time = 5e-5;
range_min = 0.15;
range_max = 3.00;

parameters_Lidar = [angle_min
    angle_max
    angle_increment
    time_increment
    scan_time
    range_min
    range_max];

% Modify parameters for batch processing
if ~isempty(p_bp)
    for k = 1:size(p_bp,2)
        parameters_Lidar(p_bp(1,k)) = p_bp(2,k);
    end
end
end

