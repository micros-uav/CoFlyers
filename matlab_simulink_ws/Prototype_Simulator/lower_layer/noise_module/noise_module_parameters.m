function parameters_noise = noise_module_parameters(vFlock,p_bp)
%MEASURE_MODEL_PARAMETERS 此处显示有关此函数的摘要
%   此处显示详细说明
parameters_noise = vFlock/10;
% parameters_measure = 0;

% Modify parameters for batch processing
if ~isempty(p_bp)
    for k = 1:size(p_bp,2)
        parameters_noise(p_bp(1,k)) = p_bp(2,k);
    end
end
end

