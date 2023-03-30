function [data] = motion_module_get_params_for_visual(parameters_motion,flag_motion_model)
%MOTION_MODULE_GET_PARAMS_FOR_VISUAL Summary of this function goes here
%   Detailed explanation goes here
[parameters_motion_sub] = motion_module_parameters_deal(parameters_motion,flag_motion_model);
switch flag_motion_model
    case 0 % point-mass
        data = [];
    case 1 % quadcopter
        data = qm_get_params_for_visual(parameters_motion_sub);
end

end

