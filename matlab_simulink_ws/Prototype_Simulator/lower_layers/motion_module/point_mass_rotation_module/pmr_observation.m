function states_ob = pmr_observation(states,dstates)
%PMR_OBSERVATION Summary of this function goes here
%   Detailed explanation goes here

states_ob = [states;dstates(4:6,:)];
end

