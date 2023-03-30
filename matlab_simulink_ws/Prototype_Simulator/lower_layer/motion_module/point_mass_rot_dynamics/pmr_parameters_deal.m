function [speed_max,...
    theta_max,...
    time_constant_pos_ctrl] = pmr_parameters_deal(parameters_motion)
%POINT_MASS_ROT_MODEL_PARAMETERS_DEAL 

count = 1;
speed_max                = parameters_motion(count); count = count + 1;
theta_max                = parameters_motion(count); count = count + 1;
time_constant_pos_ctrl                = parameters_motion(count); count = count + 1;

end

