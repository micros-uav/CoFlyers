function states = noise_module_add_noise(states)
%MEASURE_MODEL_ADD_NOISE 
%   
file_name_param = 'noise_module_parameters';
[~,str_core] = get_multi_core_value();
fun_params = str2func([file_name_param, str_core]);

[velocity_noise] = fun_params();

states(4:6,:) = states(4:6,:) + normrnd(0,velocity_noise,[3,size(states,2)]);
end

