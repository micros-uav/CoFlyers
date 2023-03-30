function [v_max,a_max,time_constant_p_ctrl,...
    time_constant_v_ctrl] = point_mass_model_parameters_deal(parameters_motion)
%POINT_MASS_MODEL_PARAMETERS_DEAL 此处显示有关此函数的摘要
%   此处显示详细说明
count = 1;
v_max                = parameters_motion(count); count = count + 1;
a_max                = parameters_motion(count); count = count + 1;
time_constant_p_ctrl = parameters_motion(count); count = count + 1;
time_constant_v_ctrl = parameters_motion(count); count = count + 1;
end

