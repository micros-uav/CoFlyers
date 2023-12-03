function [states,dstates] = pmr_dynamics_update(t, states,commands_bottom,sample_time_motion)
%PM_DYNAMICS_UPDATE Summary of this function goes here
%   Detailed explanation goes here


vxyz_ds = commands_bottom(1:3,:);

%%% Numerical intergration, first-order

dstates = [vxyz_ds;zeros(size(vxyz_ds))];
states(1:3,:) = states(1:3,:) + vxyz_ds * sample_time_motion;
states(4:6,:) = vxyz_ds;

end

