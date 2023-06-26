function states = noise_module_add_noise(states)
%MEASURE_MODEL_ADD_NOISE 
%   

[velocity_noise] = noise_module_parameters();

states(4:6,:) = states(4:6,:) + normrnd(0,velocity_noise,[3,size(states,2)]);
end

