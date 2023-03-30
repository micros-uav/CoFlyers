clc;close all;clear;
%% Settings of batch processing
flag_use_parallel = true;
repeat_times         = 1;    %Times of experimental repetitions used to average multiple simulations
%%%  Get the value combinations of parameters for batch processing.
parameters_bp_s = get_value_combinations();
%% Run batch processing
results = run_batch_processing(flag_use_parallel,...
    repeat_times, parameters_bp_s);
%% Visualization
%
parameters_bp_lb  = [0  ,0];
parameters_bp_ub  = [14,16];
parameters_bp_num = [29,33];
param1_array    = linspace(parameters_bp_lb(1),parameters_bp_ub(1),parameters_bp_num(1));
param2_array    = linspace(parameters_bp_lb(2),parameters_bp_ub(2),parameters_bp_num(2));

%
result_ = reshape(results,[size(results,1),parameters_bp_num]);
result_ = permute(result_,[2:length(size(result_)),1]);
labels = {'Velocity noise (m/s)','Control sampling time (s)','\phi_{Corr}'};
ff = result_(:,:,1)';
% [myFigure,myAxes] = plot_model_draw_monte_carlo_2D(param1_array,param2_array,ff,labels,0.4,0.3);
figure;
surf(param1_array,param2_array,ff)
xlabel(labels{1})
ylabel(labels{2})
zlabel(labels{3})
view([150,30])
box on
%% Get the value combinations of parameters for batch processing
function parameters_bp_s = get_value_combinations()
module_order = [3, 3];
param_order  = [5, 6];
parameters_bp_lb  = [0  ,0];
parameters_bp_ub  = [14,16];
parameters_bp_num = [29,33];

param1_array    = linspace(parameters_bp_lb(1),parameters_bp_ub(1),parameters_bp_num(1));
param2_array    = linspace(parameters_bp_lb(2),parameters_bp_ub(2),parameters_bp_num(2));
[param1_grid,param2_grid] = ndgrid(param1_array,param2_array);
params_s = [param1_grid(:),param2_grid(:)];
module_order_s = repmat(module_order,size(params_s,1),1);
param_order_s = repmat(param_order,size(params_s,1),1);

parameters_bp_s = cat(3,module_order_s,param_order_s,params_s);
parameters_bp_s = permute(parameters_bp_s,[3,2,1]);

end

%%
