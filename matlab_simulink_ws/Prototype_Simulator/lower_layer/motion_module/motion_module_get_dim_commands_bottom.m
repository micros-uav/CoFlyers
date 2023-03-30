function dim = motion_module_get_dim_commands_bottom(flag_motion_model)
%MOTION_MODULE_GET_DIM_COMMANDS_BOTTOM Summary of this function goes here
%   Detailed explanation goes here
switch flag_motion_model
    case 0 % point-mass
        dim = 3;
    case 1 % quadcopter
        dim = 4;
    case 2 % point-mass rotation
        dim = 3;
end
end

