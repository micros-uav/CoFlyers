function data = qm_get_params_for_visual(parameters_motion)
%QM_GET_PARAMS_FOR_VISUAL Summary of this function goes here
%   Detailed explanation goes here
[~,~,~,lenArm] =...
    quadcopter_model_parameters_deal(parameters_motion);

data = [lenArm];
end

