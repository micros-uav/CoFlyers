function [states,dstates, parameters_settings,parameters_motion,parameters_swarm,...
    parameters_evalue,parameters_visual,parameters_noise,parameters_map,...
    map_lines,map_grid,parameters_sensor_env] = ...
initialize_parameters_states(P_GUI, P_op, P_bp)
% P_GUI: parameters from gui
% P_op:  parameters to be optimized
% P_bp:  parameters for batch processing

%%% Settings, regarded as 1th module.
parameters_settings = P_GUI;
p_bp = find_out_params_of_mth_module_for_bp(P_bp,1);
% Modify parameters for batch processing
if ~isempty(p_bp)
    for kk = 1:size(p_bp,2)
        parameters_settings(p_bp(1,kk)) = p_bp(2,kk);
    end
end

[mode_simulation, number,time_max,sample_time_motion,sample_time_control_u,sample_time_control_b,...
    flag_save_data, time_state_save, flag_motion_model,flag_alg,flag_eva,...
        flag_plot,time_plot,flag_plot_traj,flag_follow,time_trajectory,...
        flag_save_video,video_speed,dimension_visual] =...
    parameters_setting_deal(parameters_settings);

%%% Map Definition. 2th module
p_bp = find_out_params_of_mth_module_for_bp(P_bp,2);
parameters_map = map_module_parameters(number,flag_alg,p_bp);
[map_lines,map_grid] = map_module_generate(parameters_map);
[~,xrange,yrange,~] = map_module_parameters_deal(parameters_map);

%%% Swarm parameters. 3th module
p_bp = find_out_params_of_mth_module_for_bp(P_bp,3);
parameters_swarm = swarm_module_parameters(number,map_lines,flag_alg,p_bp,P_op);

[parameters_swarm_sub] = swarm_module_parameters_deal(parameters_swarm,flag_alg);
dim_alg = 2;
switch flag_alg
    case 0
        [~, v_flock] = Vasarhelyi_module_parameters_deal(parameters_swarm_sub);
    case 1
        [~, v_flock] = Vasarhelyi_will_module_parameters_deal(parameters_swarm_sub);
    otherwise
        v_flock = 0;
        [~,~,dim_alg] = Couzin_module_parameters_deal(parameters_swarm_sub);
end

%%% Evaluation parameters. 4th module
p_bp = find_out_params_of_mth_module_for_bp(P_bp,4);
parameters_evalue   = evaluation_module_parameters(v_flock,flag_eva,p_bp);

%%% Motion parameters. 5th module
p_bp = find_out_params_of_mth_module_for_bp(P_bp,5);
parameters_motion   = motion_module_parameters(flag_motion_model, p_bp);
[~] = motion_module_parameters_deal(parameters_motion);

%%% noise parameters. 6th module
p_bp = find_out_params_of_mth_module_for_bp(P_bp,6);
parameters_noise = noise_module_parameters(v_flock,p_bp);

%%% parameters_sensor_env. 7th module
p_bp = find_out_params_of_mth_module_for_bp(P_bp,7);
flag_sensor = 0; % Lidar
parameters_sensor_env = sensor_env_module_parameters(flag_sensor,p_bp);


%%% Visualization parameters. 8th module
p_bp = find_out_params_of_mth_module_for_bp(P_bp,8);
parameters_visual = visual_module_parameters(flag_plot_traj,flag_follow, ...
    flag_save_data,flag_save_video,video_speed, ...
    time_trajectory,dimension_visual, p_bp);

%%% Initialize state

[states,dstates] = initialize_states(number, xrange, yrange, flag_motion_model,flag_alg,dim_alg);

%%% Find out the parameters of the mth module for batch processing
    function p_bp = find_out_params_of_mth_module_for_bp(P_bp,m)
        if ~isempty(P_bp)
            if size(P_bp,1) ~= 3
                p_bp = [];
                disp('Per column of P_bp must have 3 elements: m, n, value.');
                return
            end
            %
            temp = find(P_bp(1,:) == m);
            if ~isempty(temp)
                p_bp = P_bp(2:3,temp);
            else
                p_bp = [];
            end
        else
            p_bp = [];
        end
    end
end