function parameters_motion = point_mass_model_parameters()
%POINT_MASS_MODEL_PARAMETERS 此处显示有关此函数的摘要
%   此处显示详细说明
amax                 = 2;
vmax                 = 1.5;
time_constant_p_ctrl = 1;
time_constant_v_ctrl = 1;

parameters_motion = [
    vmax;
    amax;
    time_constant_p_ctrl;
    time_constant_v_ctrl];

end

