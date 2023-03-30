function states = quad_control_by_PosVel(xyzYaw_ds, vxyzYr_d_fws, reset, parameters)
%QUAD_CONTROL_BY_POSVEL 此处显示有关此函数的摘要
%   此处显示详细说明
% states: x, y, z, vx, vy, vz, yaw, roll, pitch, p, q, r
% pos_d
persistent w_states ep_pos ed_pos ei_pos ep_vel ed_vel ei_vel
num = parameters.number;
if isempty(w_states)
    w_states = [parameters.motor_init_con(:);parameters.vehicle_int_con(:)];
end

sampleTime = parameters.sampleTime;
sampleTime_base = parameters.sampleTime_base;
gravity = parameters.gravity;
mass = parameters.mass;
% Position control parameters
kp_pos = parameters.kp_pos';
kd_pos = parameters.kd_pos';
ki_pos = parameters.ki_pos';
c_d_pos = parameters.cd_filter_pos;
lb_pos = parameters.lb_pos';
ub_pos = parameters.ub_pos';
vMax_h = parameters.vMax_h;
vMax_v = parameters.vMax_v;
yaw_rate_max = parameters.yaw_rate_max;
% Velocity control parameters
kp_vel = parameters.kp_vel';
kd_vel = parameters.kd_vel';
ki_vel = parameters.ki_vel';
c_d_vel = parameters.cd_filter_vel';
lb_vel = parameters.lb_vel';
ub_vel = parameters.ub_vel';
aMax_h = parameters.aMax_h;
aMax_v = parameters.aMax_v;
%
euler_max = parameters.euler_max;
thrust_max = parameters.thrust_max;
%  Attitude control
params_att = [parameters.inertia'
    parameters.kp_att'
    parameters.kd_att'
    parameters.lenArm
    parameters.ct
    parameters.cm];
% Motor
T_re = parameters.T_re;
% quadcopter
params_quad = [parameters.mass
    parameters.inertia'
    parameters.lenArm
    parameters.ct
    parameters.cm];

states_origin = reshape(w_states(4*num+1:end),13,num);

YRP_pqr = [quaternion2Euler(states_origin(7:10,:));states_origin(11:13,:)];
xyzYaw = [states_origin(1:3,:);YRP_pqr(1,:)];
vel = states_origin(4:6,:);
%==============Position Control==============
error_pos = xyzYaw_ds - xyzYaw;
if isempty(ep_pos)
    ep_pos = error_pos;
    ed_pos = zeros(size(ep_pos));
    ei_pos = zeros(size(ep_pos));
end
[pid_out,ep_pos,ed_pos,ei_pos] = MultiPIDController_m(error_pos,reset,sampleTime, ...
    kp_pos,kd_pos,ki_pos,c_d_pos,lb_pos,ub_pos, ...
    ep_pos,ed_pos,ei_pos);
vxyzYr_ds = vxyzYr_d_fws + pid_out;
% Clamp
d_temp = sqrt(vxyzYr_ds(1,:).^2 + vxyzYr_ds(2,:).^2);
clamp_temp = d_temp > vMax_h;
temp = d_temp(clamp_temp);
vxyzYr_ds(1,clamp_temp) = vxyzYr_ds(1,clamp_temp)./temp * vMax_h;
vxyzYr_ds(2,clamp_temp) = vxyzYr_ds(2,clamp_temp)./temp * vMax_h;

% out = my_saturation(vxyzYr_ds(1:2,:),vMax_h,1); % Take very long time

d_temp = abs(vxyzYr_ds(3,:));
clamp_temp = d_temp > vMax_v;
vxyzYr_ds(3,clamp_temp) = vxyzYr_ds(3,clamp_temp)./d_temp(clamp_temp) * vMax_v;
d_temp = abs(vxyzYr_ds(4,:));
clamp_temp = d_temp > yaw_rate_max;
vxyzYr_ds(4,clamp_temp) = vxyzYr_ds(4,clamp_temp)./d_temp(clamp_temp) * yaw_rate_max;
%==============Velocity Control==============
axyzYr_ds = vxyzYr_ds;
error_vel = vxyzYr_ds(1:3,:) - vel;
if isempty(ep_vel)
    ep_vel = error_vel;
    ed_vel = zeros(size(ep_vel));
    ei_vel = zeros(size(ep_vel));
end
[pid_out,ep_vel,ed_vel,ei_vel] =  MultiPIDController_m(error_vel,reset,sampleTime, ...
    kp_vel,kd_vel,ki_vel,c_d_vel,lb_vel,ub_vel, ...
    ep_vel,ed_vel,ei_vel);
axyzYr_ds(1:3,:) = pid_out;
% Clamp
d_temp = sqrt(axyzYr_ds(1,:).^2 + axyzYr_ds(2,:).^2);
clamp_temp = d_temp > aMax_h;
temp = d_temp(clamp_temp);
axyzYr_ds(1,clamp_temp) = axyzYr_ds(1,clamp_temp)./temp * aMax_h;
axyzYr_ds(2,clamp_temp) = axyzYr_ds(2,clamp_temp)./temp * aMax_h;
d_temp = abs(axyzYr_ds(3,:));
clamp_temp = d_temp > aMax_v;
axyzYr_ds(3,clamp_temp) = axyzYr_ds(3,clamp_temp)./d_temp(clamp_temp) * aMax_v;

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

% More frequency
num_times = ceil(sampleTime/sampleTime_base);
if num_times<10
    num_times = 10;
end
dt = sampleTime/num_times;
for i = 1:num_times
    %==============Attitude control==============
    w_d = attitudeController_m(YrRPT_d,YRP_pqr,params_att);
    %==============Rotor dynamics==============
    % dw = motorsDynamics_m(t,w,w_d,T_re);
    %==============Quadcopter dynamics==============
    % sdot = quadcopterDynamics_m(t, s, w, params)
    fun = @(t,x)[motorsDynamics_m(t,x(1:4*num),w_d(:),T_re);quadcopterDynamics_m(t, x(4*num+1:end), x(1:4*num), params_quad)];
    %  Perform numerical integration
%     w_states_all = Runge_Kutta_4(fun,[dt*(i-1),dt*i],w_states);
%     w_states_all = integration_first_order(fun,[dt*(i-1),dt*i],w_states);
    w_states = numerical_integration(fun,[dt*(i-1),dt*i],w_states,4);
    states_origin = reshape(w_states(4*num+1:end),13,num);
    YRP_pqr = [quaternion2Euler(states_origin(7:10,:));states_origin(11:13,:)];
end
states = zeros(12,num);
states(1:6,:) = states_origin(1:6,:);
states(7:9,:) = quaternion2Euler(states_origin(7:10,:));
states(10:12,:) = states_origin(11:13,:);
end

