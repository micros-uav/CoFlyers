function parameters_motion = pmr_parameters()
%POINT_MASS_ROT_MODEL_PARAMETERS 

speed_max                 = 3;       % maximum speed
theta_max                 = 40*pi/180;      % maximum angular speed
time_constant_pos_ctrl = 0.1; % convergence time of speed

parameters_motion = [
    speed_max;
    theta_max;
    time_constant_pos_ctrl;];

end

