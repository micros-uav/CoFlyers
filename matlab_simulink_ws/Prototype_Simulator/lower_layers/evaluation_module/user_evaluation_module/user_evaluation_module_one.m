function values =user_evaluation_module_one(t, states, map3d_faces, map3d_struct)
%USER_EVALUATION_MODULE_ONE 
% Automatically generated once by read_parameter_xml.m
% This function will be called by evaluation_module_one.m

% Parameters only be generated once by read_parameter_xml.m.
% If you change the parameters of your evaluation submodule, you need to
% get parameters by user_evaluation_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
	file_name_param = 'user_evaluation_module_parameters';
	[~,str_core] = get_multi_core_value();
	fun_params = str2func([file_name_param,str_core]);
end
[v_flock] = fun_params();

%
speed = sqrt(sum(states(4:6,:).^2,1));
phi_corr = norm(mean(states(4:6,:)./speed,2));
phi_vel = mean(speed)/0.1;
values = [phi_corr;phi_vel];




end