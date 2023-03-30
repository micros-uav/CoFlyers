function states_ob = qm_observation(states,dstates)
%QM_OBSERVATION Summary of this function goes here
%   Detailed explanation goes here
number = size(states,2);
states_ob = zeros(12,number);
states_ob(1:6,:) = states(1:6,:);
states_ob(7:9,:) = dstates(4:6,:);
states_ob(10:12,:) = quaternion2Euler(states(7:10,:));
end

