clear qm_control_by_PosVel
flag_motion_model = 1;
sample_time_control = 0.1;
sample_time_motion = 0.0025;
parameters_motion   = motion_model_parameters(flag_motion_model,sample_time_control,sample_time_motion);

states_series = zeros(17,10/sample_time_motion);
states = [0;0;1;0;0;0;1;0;0;0;0;0;0;1300;1300;1300;1300];
for i = 1:10/sample_time_motion
    posDesired = [states(1:3);0];
    velDesired = [0.2;0;0;0];
    states = motion_model_update(states, posDesired, velDesired, parameters_motion,flag_motion_model);
    
    states_series(:,i) = states;
end
figure
plot(states_series(4,:))    