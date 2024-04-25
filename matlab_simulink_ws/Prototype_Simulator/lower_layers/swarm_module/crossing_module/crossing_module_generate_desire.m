function [command_upper_s,control_mode_s] =crossing_module_generate_desire(t, states, sample_time, sensor_data_s, map3d_struct, terrain, terrain_params)
%CROSSING_MODULE_GENERATE_DESIRE Generate the desired position and velocity
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
% get parameters by crossing_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
	file_name_param = 'crossing_module_parameters';
	[~,str_core] = get_multi_core_value();
	fun_params = str2func([file_name_param,str_core]);
end

[r_com,...
k,...
r_rep,...
p_rep,...
r_att,...
p_att,...
r_safe,...
v_flock,...
r_visual,...
p_visual,...
dir_task,...
id_informed,...
flag_agds,...
r_agds,...
r_safe_agds,...
x_finish,...
x_stop,...
height,...
v_max] = fun_params();

%
number = size(states,2);
command_upper_s = zeros(12,number);
command_upper_s(1:3,:) = states(1:3,:);
control_mode_s = uint8(zeros(1,number))+7;

% Traverse every agent
for id  = 1:number
	pos_desired_id = [states(1:3,id);0];
	pos_desired_id(3) = 1;
	vel_desired_id = [mean(states(4:5,:), 2);0;0];
	vel_desired_id(1:2) = 0.1*vel_desired_id(1:2)/norm(vel_desired_id(1:2));
	acc_desired_id = [0;0;0;0];
	command_upper_s(:,id) = [pos_desired_id;vel_desired_id;acc_desired_id];
	% Get sensor data
	if ~isempty(sensor_data_s)
		range_s = sensor_data_s(1,:,id);
		psi_s = sensor_data_s(2,:,id);
		phi_s = sensor_data_s(3,:,id);
	end
end


end