function [states,dstates] = qm_dynamics_update(states, commands_bottom,parameters_motion,sample_time_motion)
%QM_DYNAMICS_UPDATE Summary of this function goes here
%   Detailed explanation goes here

[gravity,inertia,mass,lenArm,vMax_h,vMax_v,...
    yaw_rate_max,aMax_h,aMax_v,euler_max,thrust_max,ct,cm,...
    T_re,kp_att,kd_att,kp_pos,ki_pos,kd_pos,c_d_pos,lb_pos,ub_pos,...
    kp_vel,ki_vel,kd_vel,c_d_vel,lb_vel,ub_vel] =...
    quadcopter_model_parameters_deal(parameters_motion);

params_quad = [mass
    inertia
    lenArm
    ct
    cm];

YrRPT_d = commands_bottom;

%==============Attitude control==============
YRP_pqr = [quaternion2Euler(states(7:10,:));states(11:13,:)];
params_att = [inertia
    kp_att
    kd_att
    lenArm
    ct
    cm];
w_d = attitudeController_m(YrRPT_d,YRP_pqr,params_att);

%==============Dynamics==================
sdot = quadcopterDynamics_m(0, states(1:13,:), states(14:17,:), params_quad);
wdot = motorsDynamics_m(0,states(14:17,:),w_d,T_re);
dstates = [sdot;wdot];
states = states + dstates*sample_time_motion;

end

