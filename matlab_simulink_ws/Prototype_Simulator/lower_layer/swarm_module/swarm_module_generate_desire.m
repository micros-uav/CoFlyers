function [command_upper_s,control_mode_s,data_swarm_for_visual] = ...
    swarm_module_generate_desire(t,states,parameters_swarm,map_lines,map_grid,parameters_sensor_env,flag_alg,sample_time)
%FLOCKING_MODEL_GENERATE_DESIRE Generate the desired position and velocity
% according to the current flocking states and model parameters
%   point-mass: state = [x; y; z; vx; vy; vz; ax; ay; az]
%   quadcopter: state = [x; y; z; vx; vy; vz; ax; ay; az; yaw; roll; Pitch];
% control_mode:
% TAKEOFF_TYPE = 2;
% HOVER_TYPE = 3;
% LAND_TYPE = 4;
% POSITION_CONTROL_TYPE = 5;
% VELOCITY_CONTROL_TYPE = 6;
% VELOCITY_HORIZONTAL_CONTROL_TYPE = 7;

[parameters_swarm_sub] = swarm_module_parameters_deal(parameters_swarm,flag_alg);
data_swarm_for_visual = [];
switch flag_alg
    case 0 
        [command_upper_s,control_mode_s] =...
            Vasarhelyi_module_generate_desire(t,states,parameters_swarm_sub,map_lines,map_grid,parameters_sensor_env);

    case 1 
        [command_upper_s,control_mode_s,will_s_for_visual] =...
            Vasarhelyi_will_module_generate_desire(t,states,parameters_swarm_sub,map_lines,map_grid,parameters_sensor_env,sample_time);
        data_swarm_for_visual = will_s_for_visual;

    case 2
        [command_upper_s,control_mode_s] =...
            Couzin_module_generate_desire(t,states,parameters_swarm_sub,map_lines,map_grid,parameters_sensor_env);
    otherwise
        command_upper_s = [];
        control_mode_s = [];
end

end


