function performances_time_average = model_swarm(parameters_gui, parameters_op, parameters_bp, re)
%MODEL_SWARM Combines the modules in lower layer into one module
%INPUT:
%   mode_simulation: =0 when rapid prototyping, =1 when auto-tuning, =2
%   when batch processing
%   parameters_gui: parameters from gui.
%   parameters_op: parameters to be optimized.
%   parameters_bp: parameters for batch processing.
%   re: mask the number of repetitions of the simulation
%   Non-empty parameter set in upper layer will modify the corresponding parameters in lower layer.
%OUTPUT:
%   performances_time_average: the time averages of performances

%========================Initialization==========================
%%% get parameters from the pre-defined file
[states,dstates, parameters_settings,parameters_motion,parameters_swarm,...
    parameters_evalue,parameters_visual,parameters_noise,parameters_map,...
    map_lines,map_grid,parameters_sensor_env] = initialize_parameters_states(parameters_gui,parameters_op, ...
    parameters_bp);

clear qm_control
%Get setting from inputs
[mode_simulation, number,time_max,sample_time_motion,sample_time_control_u,sample_time_control_b,...
    flag_save_data,time_state_save, flag_motion_model,flag_alg, flag_eva, flag_plot,time_plot] =...
    parameters_setting_deal(parameters_settings);
sample_time_control_u = max(sample_time_control_u,sample_time_motion);
sample_time_control_b = max(sample_time_control_b,sample_time_motion);

% step size of saving trajectories
interval_state_save = max(floor(time_state_save/sample_time_motion),1); 
% file name
time_now_string = char(datetime('now','TimeZone','local','Format','y_M_d_H_m_s_SSS'));
data_save_dir_name = get_dir_name_from_mode(mode_simulation);

%%% Initialize states
count = 1;
t = 0;
states_ob = motion_module_observation(states,dstates,flag_motion_model);
states_m = noise_module_add_noise(states_ob,parameters_noise);
[commands_upper,control_mode_s,data_swarm_for_visual] =...
    swarm_module_generate_desire(t,states_m,parameters_swarm,map_lines,map_grid,parameters_sensor_env, flag_alg,sample_time_control_u);
[values,values_for_visual] = evaluation_module_one(states,parameters_evalue,map_grid,parameters_map,flag_eva);
commands_bottom = motion_module_bottom_control(states, commands_upper,...
    parameters_motion,control_mode_s, flag_motion_model, sample_time_control_b);
%%% Initialize data series
count_data_save = 1;
iter_max = floor(time_max/sample_time_motion)+1;
num_data_save = ceil(iter_max/interval_state_save);
states_ob_series = zeros([size(states_ob),num_data_save]);
states_ob_series(:,:,count_data_save) = states_ob;
values_series = zeros([length(values),iter_max]);
values_series(:,count) = values;
time_series = (0:iter_max-1)*sample_time_motion;

% Initialize figure, axes and video
if flag_plot
    visual_module_draw_figures(parameters_visual, parameters_motion,parameters_map,...
        states, time_series(1), states_ob_series(:,:,1), time_series(1), values_series(:,1),...
        data_swarm_for_visual, values_for_visual,...
        map_lines, mode_simulation, 0, flag_motion_model, flag_alg, flag_eva);
end

%=========================Iteraction===========================
rate_upper   = ceil(sample_time_control_u/sample_time_motion);
rate_bottom = ceil(sample_time_control_b/sample_time_motion);
rate_draw     = ceil(time_plot/sample_time_motion);
% loop
for t = sample_time_motion:sample_time_motion:time_max
    % Next iteration
    count = count + 1;
    % Get desired position and velocity from flocking rules
    if mod(count,rate_upper) == 0
        [commands_upper,control_mode_s,data_swarm_for_visual] =...
            swarm_module_generate_desire(t,states_m,parameters_swarm,map_lines,map_grid,...
            parameters_sensor_env, flag_alg,sample_time_control_u);
    end
    
    % Bottom control
    if mod(count,rate_bottom) == 0
        commands_bottom = motion_module_bottom_control(states, commands_upper,...
            parameters_motion,control_mode_s, flag_motion_model, sample_time_control_b);
    end

    % Dynamics and kinematics simulation 
    states = motion_module_update_dynamics(states,commands_bottom,...
        parameters_motion, flag_motion_model, sample_time_motion);
    
    % Observation
    states_ob = motion_module_observation(states,dstates,flag_motion_model);

    % Add noise to states_ob
    states_m = noise_module_add_noise(states_ob,parameters_noise);

    % Calculate the performance of current frame
    [values, values_for_visual] = evaluation_module_one(states_ob,parameters_evalue,map_grid,parameters_map,flag_eva);
    
    % Save data
    if mod(count-1,interval_state_save) == 0
        count_data_save = count_data_save + 1;
        states_ob_series(:,:,count_data_save)    = states_ob;
        %     states_m_series(:,:,count_data_save)  = states_m;
    end
    values_series(:,count)      = values;
    time_series(count) = t;
    
    % Plot 
    if flag_plot
        if mod(count,rate_draw) == 0
            visual_module_draw_figures(parameters_visual, parameters_motion,parameters_map,...
                states_ob, time_series(1:interval_state_save:count), states_ob_series(:,:,1:count_data_save),...
                time_series(1:count), values_series(:,1:count),...
                data_swarm_for_visual,values_for_visual,...
                map_lines, mode_simulation, 1, flag_motion_model, flag_alg, flag_eva);
        end
    end
end
if mod(count-1,interval_state_save) ~= 0
        states_ob_series(:,:,count_data_save)    = states_ob;
end
% Calculate the performance of the whole process
performances_time_average = evaluation_module_average(values_series,parameters_evalue,flag_eva);
% Plot values, final trajectory and close Video
if flag_plot
    visual_module_draw_figures(parameters_visual, parameters_motion,parameters_map,...
        states_ob, time_series(1:interval_state_save:end), states_ob_series, time_series, values_series,...
        data_swarm_for_visual, values_for_visual,...
        map_lines, mode_simulation, 2, flag_motion_model, flag_alg, flag_eva);
end

% Save data
if flag_save_data
    t = getCurrentTask();
    id = 1;
    if ~isempty(t)
        id = t.ID;
    end
    save([data_save_dir_name,'/mats/data_',time_now_string,'_w',num2str(id),'_re',num2str(re),'.mat']);
end
% 

end

