function dim = motion_module_get_dim_commands_bottom(flag_motion_model)
%MOTION_MODULE_GET_DIM_COMMANDS_BOTTOM Summary of this function goes here
%   Detailed explanation goes here
switch flag_motion_model
    case 'point_mass' % point-mass
        dim = 3;
    case 'quadcopter' % quadcopter
        dim = 4;
    case 'point_mass_rotation' % point-mass rotation
        dim = 3;
    otherwise
        dim = 3;
end
end

