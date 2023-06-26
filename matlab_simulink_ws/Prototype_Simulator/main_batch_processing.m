clc;close all;clear;
%% Settings of batch processing
flag_use_parallel   = false;
type_parallel         = 1;      % 1: Parallel computation along parameter combinations 2: Parallel computation along repeated simulations
repeat_times         = 1;    %Times of experimental repetitions used to average multiple simulations
%%%  Get the value combinations of parameters for batch processing.
parameters_bp_s = get_value_combinations();
%% Run batch processing
results = run_batch_processing(flag_use_parallel, type_parallel, repeat_times, parameters_bp_s);

%% Get the value combinations of parameters for batch processing
function parameters_bp_s = get_value_combinations()
% module_order = [6, 1]; %Noise module and motion module
% param_order  = [1, 5];
param_name = ["CoFlyers.noise.velocity_noise", "CoFlyers.sample_time_control_upper"];
parameters_bp_lb  = [0  ,0.01];
parameters_bp_ub  = [0.5,1.01];
parameters_bp_num = [26,26];

param1_array    = linspace(parameters_bp_lb(1),parameters_bp_ub(1),parameters_bp_num(1));
param2_array    = linspace(parameters_bp_lb(2),parameters_bp_ub(2),parameters_bp_num(2));
[param1_grid,param2_grid] = ndgrid(param1_array,param2_array);
params_s = [param1_grid(:),param2_grid(:)];

% 
parameters_bp_s = [];
for i = 1:size(params_s,1)
    parameters_bp = struct();
    parameters_bp.param_name_s = param_name;
    parameters_bp.param_value_s = arrayfun(@(x)string(x),params_s(i,:));
    parameters_bp_s = [parameters_bp_s,parameters_bp];
end
end