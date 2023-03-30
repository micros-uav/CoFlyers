function parameters_motion = motion_module_parameters(flag_motion_model,p_bp)
%MOTION_MODEL_PARAMETERS 此处显示有关此函数的摘要
%   此处显示详细说明

global_params = [];

switch flag_motion_model
    case 0
        parameters_motion = point_mass_model_parameters();
    case 1
        parameters_motion = quadcopter_model_parameters();
    case 2
        parameters_motion = pmr_parameters();
    otherwise
        parameters_motion = [];
end

parameters_motion = [global_params;parameters_motion];

% Modify parameters for batch processing
if ~isempty(p_bp)
    for k = 1:size(p_bp,2)
        parameters_motion(p_bp(1,k)) = p_bp(2,k);
    end
end

end