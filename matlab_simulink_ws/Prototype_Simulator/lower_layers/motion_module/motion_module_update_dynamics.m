function [states,dstates] = motion_module_update_dynamics(t, states,commands_bottom, flag_motion_model, sample_time_motion)
%MOTION_MODEL_NEXT_FRAME 


switch flag_motion_model
    case 0 % point-mass
        [states,dstates] = pm_dynamics_update(t, states,commands_bottom,sample_time_motion);
    case 1 % quadcopter
        [states,dstates] = qm_dynamics_update(t, states, commands_bottom,sample_time_motion);
    case 2 % point-mass rotation
        [states,dstates] = pmr_dynamics_update(t, states, commands_bottom,sample_time_motion);
end

end

