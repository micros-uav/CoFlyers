function [states,dstates] = motion_module_update_dynamics(states,commands_bottom,parameters_motion, flag_motion_model, sample_time_motion)
%MOTION_MODEL_NEXT_FRAME 

[parameters_motion_sub] = motion_module_parameters_deal(parameters_motion,flag_motion_model);

switch flag_motion_model
    case 0 % point-mass
        [states,dstates] = pm_dynamics_update(states,commands_bottom,parameters_motion_sub,sample_time_motion);
    case 1 % quadcopter
        [states,dstates] = qm_dynamics_update(states, commands_bottom,parameters_motion_sub,sample_time_motion);
    case 2 % point-mass rotation
        [states,dstates] = pmr_dynamics_update(states, commands_bottom,parameters_motion_sub,sample_time_motion);
end

end

