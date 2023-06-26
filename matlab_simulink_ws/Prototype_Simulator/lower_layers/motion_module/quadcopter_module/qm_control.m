function commands_bottom = qm_control(t, states, xyzYaw_ds, vxyzYr_d_fws, axyzYr_d_fws, reset, ...
    control_mode_s,sample_time_control)
%QUAD_CONTROL_BY_POSVEL 此处显示有关此函数的摘要
%   此处显示详细说明
% states: x, y, z, vx, vy, vz, yaw, roll, pitch, p, q, r
% pos_d
%   control_mode judges the control mode by binary bit 
%    0b000001: add position control, horizontal
%    0b000010: add velocity feedforword, horizontal
%    0b000100: add acceleration feedforword, horizontal
%    0b001000: add position control, vertical
%    0b010000: add velocity feedforword, vertical
%    0b100000: add acceleration feedforword, vertical

persistent ep_pos ed_pos ei_pos ep_vel ed_vel ei_vel
num = size(states,2);

[gravity,...
inertia,...
mass,...
len_arm,...
v_max_h,...
v_max_v,...
yaw_rate_max,...
a_max_h,...
a_max_v,...
euler_max,...
thrust_max,...
ct,...
cm,...
T_rotor_inverse,...
kp_att,...
kd_att,...
kp_pos,...
ki_pos,...
kd_pos,...
cd_filter_pos,...
lb_pos_pid,...
ub_pos_pid,...
kp_vel,...
ki_vel,...
kd_vel,...
cd_filter_vel,...
lb_vel_pid,...
ub_vel_pid] = quadcopter_module_parameters();

YRP_pqr = [quaternion2Euler(states(7:10,:));states(11:13,:)];
xyzYaw = [states(1:3,:);YRP_pqr(1,:)];
vel = states(4:6,:);

%==============Position Control==============
error_pos = xyzYaw_ds - xyzYaw;
if isempty(ep_pos) || t==0
    ep_pos = error_pos;
    ed_pos = zeros(size(ep_pos));
    ei_pos = zeros(size(ep_pos));
end
[pid_out,ep_pos,ed_pos,ei_pos] = MultiPIDController_m(error_pos,reset,sample_time_control, ...
    kp_pos,kd_pos,ki_pos,cd_filter_pos,lb_pos_pid,ub_pos_pid, ...
    ep_pos,ed_pos,ei_pos);
vxyzYr_ds = pid_out;

%%% no pos hori control
vxyzYr_ds([1:2,4],bitand(control_mode_s,0b000001) == 0) = 0;

%%% no pos vert control
vxyzYr_ds(3,bitand(control_mode_s,0b001000) == 0) = 0;

%============Velocity Feedforword=============
no_v_hori_fw = bitand(control_mode_s,0b000010) == 0;
no_v_vert_fw = bitand(control_mode_s,0b001000) == 0;
vxyzYr_d_fws([1,2,4],no_v_hori_fw) = 0;
vxyzYr_d_fws(3,no_v_vert_fw) = 0;
vxyzYr_ds = vxyzYr_ds + vxyzYr_d_fws;
%==============Velocity Control==============
% Clamp
d_temp = sqrt(vxyzYr_ds(1,:).^2 + vxyzYr_ds(2,:).^2);
clamp_temp = d_temp > v_max_h;
temp = d_temp(clamp_temp);
vxyzYr_ds(1,clamp_temp) = vxyzYr_ds(1,clamp_temp)./temp * v_max_h;
vxyzYr_ds(2,clamp_temp) = vxyzYr_ds(2,clamp_temp)./temp * v_max_h;
d_temp = abs(vxyzYr_ds(3,:));
clamp_temp = d_temp > v_max_v;
vxyzYr_ds(3,clamp_temp) = vxyzYr_ds(3,clamp_temp)./d_temp(clamp_temp) * v_max_v;
d_temp = abs(vxyzYr_ds(4,:));
clamp_temp = d_temp > yaw_rate_max;
vxyzYr_ds(4,clamp_temp) = vxyzYr_ds(4,clamp_temp)./d_temp(clamp_temp) * yaw_rate_max;

axyzYr_ds = vxyzYr_ds;
error_vel = vxyzYr_ds(1:3,:) - vel;
if isempty(ep_vel) || t==0
    ep_vel = error_vel;
    ed_vel = zeros(size(ep_vel));
    ei_vel = zeros(size(ep_vel));
end
[pid_out,ep_vel,ed_vel,ei_vel] =  MultiPIDController_m(error_vel,reset,sample_time_control, ...
    kp_vel,kd_vel,ki_vel,cd_filter_vel,lb_vel_pid,ub_vel_pid, ...
    ep_vel,ed_vel,ei_vel);
axyzYr_ds(1:3,:) = pid_out;


%%% no vel hori control
axyzYr_ds([1,2,4],bitand(control_mode_s,0b000011) == 0) = 0;
%%% no vel vert control
axyzYr_ds(3,bitand(control_mode_s,0b011000) == 0) = 0;


%============Acceleration Feedforword=============
no_a_hori_fw = bitand(control_mode_s,0b000100) == 0;
no_a_vert_fw = bitand(control_mode_s,0b100000) == 0;
axyzYr_d_fws([1,2,4],no_a_hori_fw) = 0;
axyzYr_d_fws(3,no_a_vert_fw) = 0;
axyzYr_ds = axyzYr_ds + axyzYr_d_fws;


% Clamp
d_temp = sqrt(axyzYr_ds(1,:).^2 + axyzYr_ds(2,:).^2);
clamp_temp = d_temp > a_max_h;
temp = d_temp(clamp_temp);
axyzYr_ds(1,clamp_temp) = axyzYr_ds(1,clamp_temp)./temp * a_max_h;
axyzYr_ds(2,clamp_temp) = axyzYr_ds(2,clamp_temp)./temp * a_max_h;
d_temp = abs(axyzYr_ds(3,:));
clamp_temp = d_temp > a_max_v;
axyzYr_ds(3,clamp_temp) = axyzYr_ds(3,clamp_temp)./d_temp(clamp_temp) * a_max_v;

%==============Acceleration to YrRPT==============
cosYaw = cos(YRP_pqr(1,:));
sinYaw = sin(YRP_pqr(1,:));
YrRPT_d = zeros(4,num);
YrRPT_d(1,:) = axyzYr_ds(4,:); % Desired yaw rate
YrRPT_d(2,:) = 1/gravity * (axyzYr_ds(1,:).*sinYaw - axyzYr_ds(2,:).*cosYaw); % Desired roll
YrRPT_d(3,:) = 1/gravity * (axyzYr_ds(1,:).*cosYaw + axyzYr_ds(2,:).*sinYaw); % Desired pitch
YrRPT_d(4,:) = (axyzYr_ds(3,:) + gravity) * mass; % Desired thrust
%Clamp
d_temp = abs(YrRPT_d(2,:));
clamp_temp = d_temp > euler_max;
YrRPT_d(2,clamp_temp) = YrRPT_d(2,clamp_temp)./d_temp(clamp_temp) * euler_max;
d_temp = abs(YrRPT_d(3,:));
clamp_temp = d_temp > euler_max;
YrRPT_d(3,clamp_temp) = YrRPT_d(3,clamp_temp)./d_temp(clamp_temp) * euler_max;
d_temp = abs(YrRPT_d(4,:));
clamp_temp = d_temp > thrust_max;
YrRPT_d(4,clamp_temp) = YrRPT_d(4,clamp_temp)./d_temp(clamp_temp) * thrust_max;


commands_bottom = YrRPT_d;



end

