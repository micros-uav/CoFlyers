function out = visual_module_get_activate_BD()
%VISUAL_MODULE_GET_ACTIVATE_BD 
%   

[activate_plot,...
time_interval_plot,...
activate_trajectory,...
follow_agent,...
activate_save_figure,...
activate_save_video,...
dim_visual,...
time_interval_trajectory,...
video_speed,...
x_range,...
y_range,...
z_range,...
legend_name,...
font_size,...
font_size_sub,...
marker_size,...
background_color,...
activate_BD_1] = visual_module_parameters();

out = activate_BD_1>0 && activate_plot>0;

end

