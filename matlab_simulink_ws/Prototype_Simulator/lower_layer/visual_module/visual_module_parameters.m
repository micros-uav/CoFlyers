function parameters_visual = visual_module_parameters(flag_plot_traj,flag_follow,...
    flag_save_data,flag_save_video,video_speed, ...
    time_trajectory,dimension, p_bp)
%PLOT_MODEL_PARAMETERS 


parameters_visual = [flag_plot_traj;
    flag_follow;
    flag_save_data;
    flag_save_video;
    dimension;
    time_trajectory;
    video_speed];

% Modify parameters for batch processing
if ~isempty(p_bp)
    for k = 1:size(p_bp,2)
        parameters_visual(p_bp(1,k)) = p_bp(2,k);
    end
end
end

