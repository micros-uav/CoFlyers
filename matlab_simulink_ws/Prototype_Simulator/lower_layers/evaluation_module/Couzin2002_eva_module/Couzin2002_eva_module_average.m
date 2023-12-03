function values =Couzin2002_eva_module_average(time_series, values_series)
%COUZIN2002_EVA_MODULE_AVERAGE 
% Automatically generated once by read_parameter_xml.m
% This function will be called by evaluation_module_average.m

% Parameters only be generated once by read_parameter_xml.m.
% If you change the parameters of your evaluation submodule, you need to
% get parameters by Couzin2002_eva_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
	file_name_param = 'Couzin2002_eva_module_parameters';
	[~,str_core] = get_multi_core_value();
	fun_params = str2func(strcat(file_name_param,str_core));
end
fun_params();

%
p_group = mean(values_series(1,:));
m_group = mean(values_series(2,:));
F = p_group*m_group;
values = [F;p_group;m_group];




end