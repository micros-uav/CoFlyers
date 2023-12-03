function values =PSO_eva_module_average(time_series, values_series)
%PSO_EVA_MODULE_AVERAGE 
% Automatically generated once by read_parameter_xml.m
% This function will be called by evaluation_module_average.m

% Parameters only be generated once by read_parameter_xml.m.
% If you change the parameters of your evaluation submodule, you need to
% get parameters by PSO_eva_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
	file_name_param = 'PSO_eva_module_parameters';
	[~,str_core] = get_multi_core_value();
	fun_params = str2func(strcat(file_name_param,str_core));
end
[fit_f] = fun_params();

%
f_min = values_series(1,end);
F = f_min;
values = [F;f_min];




end