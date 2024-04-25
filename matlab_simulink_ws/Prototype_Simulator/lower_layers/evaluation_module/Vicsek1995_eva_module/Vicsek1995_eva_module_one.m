function values =Vicsek1995_eva_module_one(t, states, map3d_faces, map3d_struct, terrain, terrain_params)
%VICSEK1995_EVA_MODULE_ONE 
% Automatically generated once by read_parameter_xml.m
% This function will be called by evaluation_module_one.m

% Parameters only be generated once by read_parameter_xml.m.
% If you change the parameters of your evaluation submodule, you need to
% get parameters by Vicsek1995_eva_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
	file_name_param = 'Vicsek1995_eva_module_parameters';
	[~,str_core] = get_multi_core_value();
	fun_params = str2func([file_name_param,str_core]);
end
[v_flock] = fun_params();

%
phi_v = norm(mean(states(4:6,:),2))/v_flock;
values = [phi_v];




end