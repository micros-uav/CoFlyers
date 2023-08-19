function values =user_evaluation_module_average(time_series, values_series)
%USER_EVALUATION_MODULE_AVERAGE 
% Automatically generated once by read_parameter_xml.m
% This function will be called by evaluation_module_average.m

% Parameters only be generated once by read_parameter_xml.m.
% If you change the parameters of your evaluation submodule, you need to
% get parameters by user_evaluation_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
	file_name_param = 'user_evaluation_module_parameters';
	[~,str_core] = get_multi_core_value();
	fun_params = str2func(strcat(file_name_param,str_core));
end
[v_flock] = fun_params();

%
phi_corr = mean(values_series(1,:));
phi_vel = mean(values_series(2,:));
F = phi_corr*phi_vel;
values = [F;phi_corr;phi_vel];




end