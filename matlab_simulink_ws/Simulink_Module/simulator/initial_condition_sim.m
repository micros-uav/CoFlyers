
%%
number = 6;
sampleTime_base = 0.01;
sampleTime_control = 0.01;
parameters_motion = quadcopter_model_parameters();
[gravity,inertia,mass,lenArm,vMax_h,vMax_v,...
    yaw_rate_max,aMax_h,aMax_v,euler_max,thrust_max,ct,cm,...
    T_re,kp_att,kd_att,kp_pos,ki_pos,kd_pos,cd_filter_pos,lb_pos,ub_pos,...
    kp_vel,ki_vel,kd_vel,cd_filter_vel,lb_vel,ub_vel] =...
    quadcopter_model_parameters_deal(parameters_motion);

inertia = inertia';
kp_att = kp_att';
kd_att = kd_att';

kp_pos = kp_pos';
ki_pos = ki_pos';
kd_pos = kd_pos';
lb_pos = lb_pos';
ub_pos = ub_pos';

kp_vel = kp_vel';
ki_vel = ki_vel';
kd_vel = kd_vel';
lb_vel = lb_vel';
ub_vel = ub_vel';

delta_r = 1;
xrange = [-5,5];
yrange = [-5,5];
NSqrt = ceil(sqrt(number));
position = [(mod(0:number-1,NSqrt) - mod(number-1,NSqrt)/2)*delta_r;
    (floor((0:number-1)/NSqrt) - floor((number-1)/NSqrt)/2)*delta_r;
    zeros(1,number)]; 
velocity = [rand(2,number)*0.001;zeros(1,number)];
qwxyz = zeros(4,number); qwxyz(1,:) = 1;
pqr = zeros(3,number);
motor_speed = zeros(4,number);
states_omeges = [position;velocity;qwxyz;pqr;motor_speed];

% states_omeges = initialize_states(number,parameters_flocking,parameters_motion,...
%     parameters_map,1);
vehicle_int_con = states_omeges(1:13,:);
motor_init_con = states_omeges(14:17,:);

