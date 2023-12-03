function states_ob = motion_module_observation(states,dstates,...
    flag_motion_model)
%MOTION_MODULE_OBSERVATION Summary of this function goes here
%   Detailed explanation goes here
switch flag_motion_model
    case 'point_mass' % point-mass
        states_ob = pm_observation(states,dstates);
    case 'quadcopter' % quadcopter
        states_ob = qm_observation(states,dstates);
    case 'point_mass_rotation' % point-mass rotation
        states_ob = pmr_observation(states,dstates);
    otherwise
        states_ob = pm_observation(states,dstates);
end
end

