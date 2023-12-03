%% using parameters.xml to config. 
close all;

[map3d_faces, map3d_struct, model_stls, params, position0, param_simulink] =...
read_parameter_xml("../Prototype_Simulator/xml_config_files/parameters.xml");

model_stls = join(model_stls);
sim_quad = param_simulink.sim_quad;
number   = params.number;
flag_alg = params.swarm_algorithm_type;
motion_model_type = params.motion_model_type;
sample_time_control_upper = params.sample_time_control_upper;
sample_time_control_bottom = params.sample_time_control_bottom;
sample_time_motion = params.sample_time_motion;
%% 
if sim_quad == 0
    % Real-world experiment
    sample_time_base = sample_time_control_upper;
    sample_time_control_bottom =  sample_time_control_upper;
    sample_time_motion =  sample_time_control_upper;
    number_real = number;
    number_sim  = 1;        % Cannot be 0
    activate_sim = false;
elseif sim_quad == 1
    % Simple simulation
    sample_time_base = 0.0025;
    param_simulink.local_ip = '127.0.0.1';
    param_simulink.target_ip = ones(1,number)*1;
    number_real = 0 ;
    number_sim  = number;
    activate_sim = true;
else
    % VR experiment
    sample_time_base = 0.0025;
    number_real = min(param_simulink.number_real,number);
    number_sim  = max(number - number_real,0);
    activate_sim = true;
end
local_ip_sim = param_simulink.local_ip;
target_ip_array_sim = param_simulink.target_ip(number-number_sim+1:number);
local_ip = param_simulink.local_ip;
target_ip_array = param_simulink.target_ip;
position0_sim = position0(:,number-number_sim+1:end);
