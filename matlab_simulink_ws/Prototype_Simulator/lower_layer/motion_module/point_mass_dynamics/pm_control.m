function commands_bottom = pm_control(states,xyz_ds, vxyz_d_fws, ...
    axyz_d_fws, parameters_motion,control_mode,sample_time_control)
%PM_CONTROL_BY_POSVEL_2D Control the motion of the point mass through 
% velocity command or the combination of position command and velocity feedforward command
%   If mode = 0: velocity command, set xy_ds to any value
%       otherwise: the combination of position command and velocity feedforward command
%   xy_ds  = [x_ds;y_ds];
%   vxy_ds = [vx_ds;vy_ds];
%   parameters_point_mass_2D = [amax,vmax,sampleTime_ctrl,sampleTime_base];

%    0b000001: add position control, horizontal
%    0b000010: add velocity feedforword, horizontal
%    0b000100: add acceleration feedforword, horizontal
%    0b001000: add position control, vertical
%    0b010000: add velocity feedforword, vertical
%    0b100000: add acceleration feedforword, vertical

%%% Get paramters
[v_max,a_max,time_constant_p_ctrl,...
    time_constant_v_ctrl] = point_mass_model_parameters_deal(parameters_motion);

%%% Calculate desired velocity
vxyz_d = (xyz_ds - states(1:3,:))/time_constant_p_ctrl;
if bitand(control_mode,0b000001) == 0
    vxyz_d([1,2],:) = 0;
end
if bitand(control_mode,0b001000) == 0
    vxyz_d(3,:) = 0;
end

if bitand(control_mode,0b000010) > 0
    vxyz_d([1,2],:) = vxyz_d([1,2],:) + vxyz_d_fws([1,2],:);
end
if bitand(control_mode,0b001000) > 0
    vxyz_d(3,:) = vxyz_d(3,:) + vxyz_d(3,:);
end

vxyz_d_norm = sqrt(sum(vxyz_d.^2,1));
clamp_v_d = find(vxyz_d_norm >  v_max);
if ~isempty(clamp_v_d)
    vxyz_d(:,clamp_v_d) = vxyz_d(:,clamp_v_d)./vxyz_d_norm(clamp_v_d) * v_max;
end
%%% Calculate acceleration
vxyz = states(4:6,:);
axyz = (vxyz_d - vxyz)/time_constant_v_ctrl;

if bitand(control_mode,0b000011) == 0
    axyz([1,2],:) = 0;
end
if bitand(control_mode,0b011000) == 0
    axyz(3,:) = 0;
end


if bitand(control_mode,0b000100) > 0
    axyz([1,2],:) = axyz([1,2],:) + axyz_d_fws([1,2,4],:);
end

if bitand(control_mode,0b100000) > 0
    axyz(3,:) = axyz(3,:) + axyz_d_fws(3,:);
end

commands_bottom = axyz;

end

