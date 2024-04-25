function values =PSO_eva_module_one(t, states, map3d_faces, map3d_struct, terrain, terrain_params)
%PSO_EVA_MODULE_ONE 
% Automatically generated once by read_parameter_xml.m
% This function will be called by evaluation_module_one.m

% Parameters only be generated once by read_parameter_xml.m.
% If you change the parameters of your evaluation submodule, you need to
% get parameters by PSO_eva_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
	file_name_param = 'PSO_eva_module_parameters';
	[~,str_core] = get_multi_core_value();
	fun_params = str2func([file_name_param,str_core]);
end
[fit_f] = fun_params();


fit_function = str2func(fit_f);
%
pos_t = map3d_struct(1:2,1);
f_min = min(fit_function(states(1:2,:),pos_t));
values = [f_min];



end