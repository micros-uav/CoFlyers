function states_ob = pm_observation(states,dstates)
%PM_OBSERVATION Summary of this function goes here
%   Detailed explanation goes here

states_ob = [states;dstates(4:6,:)];
end

