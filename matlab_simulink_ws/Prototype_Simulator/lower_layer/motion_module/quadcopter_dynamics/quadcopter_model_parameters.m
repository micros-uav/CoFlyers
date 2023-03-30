function parameters_motion = quadcopter_model_parameters()
%QUADCOPTER_MODEL_PARAMETERS 此处显示有关此函数的摘要
%   此处显示详细说明
% number = 12;
% sampleTime_base = 0.002; %Model sample time and attitude control sample time
% sampleTime_motion = sample_time_control/10; %Model sample time and attitude control sample time
% sample_time_control = sample_time_control;       %Control Sample Time(position,velocity)
gravity = 9.81;
% Vehicle setting
inertia=[2.90833e-4,2.90833e-4,5.4e-4];
mass = 0.092; %86g + 6g
lenArm = 0.06;
vMax_h = 1.5;                   % Maximum horizontal velocity 
vMax_v = 1.0;                   % Maximum vertical velocity 
yaw_rate_max = 0.5;             % Maximum yaw velocity 
aMax_h = 2;                     % Maximum horizontal acceleration 
aMax_v = 2;                     % Maximum vertical acceleration 
euler_max = 15*pi/180;          % Maximum roll and pitch
thrust_max = 2.5*mass*gravity;  % Maximum thrust, N
% vehicle_int_con = zeros(13,number);
% deltaX = 1; 
% N2 = floor(sqrt(number));
% vehicle_int_con(7,:) = 1;
% vehicle_int_con(1,:) = (mod(0:number-1,N2) - mod(number-1,N2)/2)*deltaX;
% vehicle_int_con(2,:) = (floor((0:number-1)/N2) - floor((number-1)/N2)/2)*deltaX;

% propeller setting
% ct = 0.0107;
% cm = 7.8264e-04;
ct = 6.11e-8*2;
cm = 1.5e-9*2;
% motor setting
T_re = 20; % uint: s^(-1)
% motor_init_con = zeros(4,number);
% attitude controller
% kp_att = [0.0429,0.0429,0.0867];
% kd_att = [0.00429,0.00429,0.00867];
kp_att = [0.8725,0.8725,1.6200] * 0.1;
kd_att = [0.08725,0.08725,0.16200] * 0.2;
% position controller
kp_pos = [1,1,1,0.1];
ki_pos = [0,0,0,0];
kd_pos = [0,0,0,0];
cd_filter_pos = 0.4;
lb_pos = [-2,-2,-2,-2];
ub_pos = [2,2,2,2];
% velocity controller
kp_vel = [2.2,2.2,1];% kp_vel = [5,5,1];
ki_vel = [0.0,0.0,0];% ki_vel = [0,0,0];
kd_vel = [0,0,0];
cd_filter_vel = 0.4;
lb_vel = [-2,-2,-2];
ub_vel = [2,2,2];

parameters_motion = [gravity;
    inertia(:);
    mass; 
    lenArm;
    vMax_h;
    vMax_v;
    yaw_rate_max;
    aMax_h;
    aMax_v;
    euler_max;
    thrust_max;
    ct;
    cm;
    T_re;
    kp_att(:);
    kd_att(:);
    kp_pos(:);
    ki_pos(:);
    kd_pos(:);
    cd_filter_pos;
    lb_pos(:);
    ub_pos(:);
    kp_vel(:);
    ki_vel(:);
    kd_vel(:);
    cd_filter_vel;
    lb_vel(:);
    ub_vel(:)];


end

