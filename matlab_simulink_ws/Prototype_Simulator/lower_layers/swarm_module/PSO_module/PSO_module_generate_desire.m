function [command_upper_s,control_mode_s] =PSO_module_generate_desire(t, states, sample_time, sensor_data_s, map3d_struct)
%PSO_MODULE_GENERATE_DESIRE Generate the desired position and velocity
% Automatically generated once by read_parameter_xml.m
% This function will be called by swarms_module_generate_desire.m
%   point-mass: state = [x; y; z; vx; vy; vz; ax; ay; az]
%   quadcopter: state = [x; y; z; vx; vy; vz; ax; ay; az; yaw; roll; Pitch];
% control_mode:
% TAKEOFF_TYPE = 2;
% HOVER_TYPE = 3;
% LAND_TYPE = 4;
% POSITION_CONTROL_TYPE = 5;
% VELOCITY_CONTROL_TYPE = 6;
% VELOCITY_HORIZONTAL_CONTROL_TYPE = 7;


% Parameters only be generated once by read_parameter_xml.m.
% If you change the parameters of your swarm submodule, you need to
% get parameters by PSO_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
	file_name_param = 'PSO_module_parameters';
	[~,str_core] = get_multi_core_value();
	fun_params = str2func([file_name_param,str_core]);
end

[c_local,...
c_global,...
fit_f] = fun_params();

fit_function = str2func(fit_f);
% Get Target
pos_target_2d = zeros(2,1);
ind = find(map3d_struct(15,:) ==0);
if ~isempty(ind)
    pos_target_2d = map3d_struct(1:2,ind);
end

% Traverse every agent
number = size(states,2);
position_s_2d = states(1:2,:);
velocity_s_2d = states(4:5,:);

% Store the best global pos and local pos_s
persistent pos_best_local_s value_best_local_s pos_target_2d_pre
if t==0 || isempty(pos_best_local_s)
    pos_best_local_s = position_s_2d;
    value_best_local_s = zeros(1,number)+inf;
    pos_target_2d_pre = pos_target_2d;
end
if norm(pos_target_2d_pre - pos_target_2d) > 0
    pos_best_local_s = position_s_2d;
    value_best_local_s = zeros(1,number)+inf;
    pos_target_2d_pre = pos_target_2d;
end
value_now = fit_function(position_s_2d,pos_target_2d);
temp = value_now < value_best_local_s;
pos_best_local_s(:,temp) = position_s_2d(:,temp);
value_best_local_s(temp) = value_now(temp);
[~,ind] = min(value_best_local_s);
pos_best_global = pos_best_local_s(:,ind);
value_best_global = value_best_local_s(ind);

% Init
number = size(states,2);
command_upper_s = zeros(12,number);
command_upper_s(1:3,:) = states(1:3,:);
control_mode_s = uint8(zeros(1,number))+7;

% PSO formula
velocity_d_s_2d = velocity_s_2d +...
    c_local*(pos_best_local_s - position_s_2d) +...
    c_global*(pos_best_global - position_s_2d);

% To commands
command_upper_s(3,:) = 1;
command_upper_s(5:6,:) = velocity_d_s_2d;
command_upper_s(7,:) = 0;

end