parameters_map = map_module_parameters(6,[]);
[map_lines,map_grid] = map_module_generate(parameters_map);

%%
tic
position2D = [-1;
    -2];
yaw = 0;

parameters_Lidar = Lidar_module_parameters([]);
angles = Lidar_module_get_angles(parameters_Lidar);
ranges = Lidar_module_get_ranges(position2D,yaw,map_lines,parameters_Lidar);
toc

points = Lidar_module_get_points_cloud(position2D,yaw,map_lines,parameters_Lidar);

% %%% Fig1
% figure
% hold on;
% axis equal
% plot(map_lines([1,3],:),map_lines([2,4],:),'-k')
% plot(position2D(1),position2D(2),'*')
% plot(points(1,:),points(2,:),'o')
% 
% visualization_module_draw_Lidar(position,angles,ranges,8,0.15,3)
% % %%% Fig2
% figure;
% plot(angles,ranges);
%%
[angle_min, angle_max, angle_increment, time_increment, scan_time,...
    range_min, range_max] = Lidar_module_parameters_deal(parameters_Lidar);
dimension = 3;
position = [position2D;1];
myFigure = figure;
myAxes = axes;
axis equal;grid on;hold on;
view([-20,60])
parameters_plot = plot_model_parameters(1,[-5,5],[-5,5],[0,3],1,1,1,1);
plot_model_draw_environment(0,1,myFigure,myAxes,parameters_plot,parameters_map,map_lines);
plot_model_draw_environment(0,2,myFigure,myAxes,parameters_plot,parameters_map,map_lines);
plot_model_draw_flocking(0,1,[position;0;0;0;0;0;0;0;0;0],[],myFigure,myAxes,parameters_plot)
plot_model_draw_flocking(0,2,[position;0;0;0;0;0;0;0;0;0],[],myFigure,myAxes,parameters_plot)
visualization_module_draw_Lidar(position-[0;0;0.05],angles,ranges,range_max,range_min,dimension);
title([])