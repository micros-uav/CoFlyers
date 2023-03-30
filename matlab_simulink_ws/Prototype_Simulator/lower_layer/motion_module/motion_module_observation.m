function states_ob = motion_module_observation(states,dstates,...
    flag_motion_model)
%MOTION_MODULE_OBSERVATION Summary of this function goes here
%   Detailed explanation goes here
switch flag_motion_model
    case 0 % point-mass
        states_ob = pm_observation(states,dstates);
    case 1 % quadcopter
        states_ob = qm_observation(states,dstates);
    case 2 % point-mass rotation
        states_ob = pmr_observation(states,dstates);
end
end

