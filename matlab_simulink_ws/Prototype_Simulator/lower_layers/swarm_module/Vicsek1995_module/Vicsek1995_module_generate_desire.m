function [command_upper_s,control_mode_s] =Vicsek1995_module_generate_desire(t, states, sample_time, sensor_data_s, map3d_struct, terrain, terrain_params)
%VICSEK1995_MODULE_GENERATE_DESIRE Generate the desired position and velocity
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
% get parameters by Vicsek1995_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
	file_name_param = 'Vicsek1995_module_parameters';
	[~,str_core] = get_multi_core_value();
	fun_params = str2func([file_name_param,str_core]);
end

[r_com,...
v_flock,...
eta] = fun_params();

%
number = size(states,2);
command_upper_s = zeros(12,number);
command_upper_s(1:3,:) = states(1:3,:);
control_mode_s = uint8(zeros(1,number))+7;

% Traverse every agent
position_s_2d = states(1:2,:);
velocity_s_2d = states(4:5,:);

% Local interaction 
dis_mat_2d = sqrt((position_s_2d(1,:) - position_s_2d(1,:)').^2+...
    (position_s_2d(2,:) - position_s_2d(2,:)').^2);
connect_matrix = dis_mat_2d<r_com;
connect_matrix = reshape(connect_matrix,[1,size(connect_matrix)]);

% Alignment
velocity_d_s_2d = squeeze(mean(connect_matrix.*velocity_s_2d,2));
velocity_d_s_2d = velocity_d_s_2d./vecnorm(velocity_d_s_2d);

% Noise
direction_s = atan2(velocity_d_s_2d(2,:),velocity_d_s_2d(1,:)) + (rand(1,number)-0.5)*eta;
velocity_d_s_2d = v_flock*[cos(direction_s);sin(direction_s)];

% To commands
command_upper_s(3,:) = 1;
command_upper_s(5:6,:) = velocity_d_s_2d;
command_upper_s(7,:) = 0;

% for id  = 1:number
% 
% 
% 
% 	pos_desired_id = [states(1:3,id);0];
% 	pos_desired_id(3) = 1;
% 	vel_desired_id = [mean(states(4:5,:), 2);0;0];
% 	vel_desired_id(1:2) = 0.1*vel_desired_id(1:2)/norm(vel_desired_id(1:2));
% 	acc_desired_id = [0;0;0;0];
% 	command_upper_s(:,id) = [pos_desired_id;vel_desired_id;acc_desired_id];
% 	% Get sensor data
% 	temp = size(sensor_data_s,1)/3;
% 	range_s = sensor_data_s(1:temp,:);
% 	psi_s = sensor_data_s((temp+1):(temp*2),:);
% 	phi_s = sensor_data_s((temp*2+1):(temp*3),:);
% end


end