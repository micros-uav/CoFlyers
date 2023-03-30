function points = Lidar_module_get_points_cloud(position2D,yaw,map_lines,parameters_Lidar)
%LIDAR_MODULE_GET_POINTS_CLOUD 此处显示有关此函数的摘要
%   此处显示详细说明

angles = Lidar_module_get_angles(parameters_Lidar);
ranges = Lidar_module_get_ranges(position2D,yaw,map_lines,parameters_Lidar);

points = zeros(2,length(angles));
points(1,:) = position2D(1) + ranges.*cos(angles);
points(2,:) = position2D(2) + ranges.*sin(angles);

end

