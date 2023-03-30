function [mode_simulation,number,time_max,...
    sample_time_motion,sample_time_control_u,sample_time_control_b,...
    flag_save_data,time_state_save,flag_motion_model,flag_alg,flag_eva,...
        flag_plot,time_plot,flag_plot_traj,flag_plot_follow,time_trajectory,...
        flag_save_video,video_speed,dimension_visual] =...
        parameters_setting_deal(parameters_settings)
%GUI_PARAMETERS_DEAL 此处显示有关此函数的摘要
%   此处显示详细说明
count = 1;
mode_simulation   = parameters_settings(count); count = count + 1;
number              = parameters_settings(count); count = count + 1;
time_max            = parameters_settings(count); count = count + 1;
sample_time_motion  = parameters_settings(count); count = count + 1;
sample_time_control_u = parameters_settings(count); count = count + 1;
sample_time_control_b = parameters_settings(count); count = count + 1;
flag_save_data      = parameters_settings(count); count = count + 1;
time_state_save      = parameters_settings(count); count = count + 1;
flag_motion_model   = parameters_settings(count); count = count + 1;
flag_alg           = parameters_settings(count); count = count + 1;
flag_eva           = parameters_settings(count); count = count + 1;
flag_plot           = parameters_settings(count); count = count + 1;
time_plot           = parameters_settings(count); count = count + 1;

flag_plot_traj      = parameters_settings(count); count = count + 1;
flag_plot_follow = parameters_settings(count); count = count + 1;
time_trajectory     = parameters_settings(count); count = count + 1;
flag_save_video     = parameters_settings(count); count = count + 1;
video_speed         = parameters_settings(count); count = count + 1;
dimension_visual   = parameters_settings(count); count = count + 1;
end

