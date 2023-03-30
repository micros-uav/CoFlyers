function states = noise_module_add_noise(states,parameters_noise)
%MEASURE_MODEL_ADD_NOISE 此处显示有关此函数的摘要
%   此处显示详细说明
[sigma_velocity] = noise_module_parameters_deal(parameters_noise);

states(4:6,:) = states(4:6,:) + normrnd(0,sigma_velocity,[3,size(states,2)]);
end

