function [gravity,inertia,mass,lenArm,vMax_h,vMax_v,...
    yaw_rate_max,aMax_h,aMax_v,euler_max,thrust_max,ct,cm,...
    T_re,kp_att,kd_att,kp_pos,ki_pos,kd_pos,cd_filter_pos,lb_pos,ub_pos,...
    kp_vel,ki_vel,kd_vel,cd_filter_vel,lb_vel,ub_vel] =...
    quadcopter_model_parameters_deal(parameters_motion)
%QUADCOPTER_MODEL_PARAMETERS_DEAL 此处显示有关此函数的摘要
%   此处显示详细说明
count = 1;
gravity             = parameters_motion(count); count = count + 1;
inertia             = parameters_motion(count:count+2); count = count + 3;
mass                = parameters_motion(count); count = count + 1;
lenArm              = parameters_motion(count); count = count + 1;
vMax_h              = parameters_motion(count); count = count + 1;
vMax_v              = parameters_motion(count); count = count + 1;
yaw_rate_max        = parameters_motion(count); count = count + 1;
aMax_h              = parameters_motion(count); count = count + 1;
aMax_v              = parameters_motion(count); count = count + 1;
euler_max           = parameters_motion(count); count = count + 1;
thrust_max          = parameters_motion(count); count = count + 1;
ct                  = parameters_motion(count); count = count + 1;
cm                  = parameters_motion(count); count = count + 1;
T_re                = parameters_motion(count); count = count + 1;
kp_att              = parameters_motion(count:count+2); count = count + 3;
kd_att              = parameters_motion(count:count+2); count = count + 3;
kp_pos              = parameters_motion(count:count+3); count = count + 4;
ki_pos              = parameters_motion(count:count+3); count = count + 4;
kd_pos              = parameters_motion(count:count+3); count = count + 4;
cd_filter_pos       = parameters_motion(count); count = count + 1;
lb_pos              = parameters_motion(count:count+3); count = count + 4;
ub_pos              = parameters_motion(count:count+3); count = count + 4;
kp_vel              = parameters_motion(count:count+2); count = count + 3;
ki_vel              = parameters_motion(count:count+2); count = count + 3;
kd_vel              = parameters_motion(count:count+2); count = count + 3;
cd_filter_vel       = parameters_motion(count); count = count + 1;
lb_vel              = parameters_motion(count:count+2); count = count + 3;
ub_vel              = parameters_motion(count:count+2); count = count + 3;
end

