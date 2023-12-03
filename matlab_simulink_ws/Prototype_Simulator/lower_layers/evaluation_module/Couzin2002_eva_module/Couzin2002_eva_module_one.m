function values =Couzin2002_eva_module_one(t, states, map3d_faces, map3d_struct)
%COUZIN2002_EVA_MODULE_ONE 
% Automatically generated once by read_parameter_xml.m
% This function will be called by evaluation_module_one.m

% Parameters only be generated once by read_parameter_xml.m.
% If you change the parameters of your evaluation submodule, you need to
% get parameters by Couzin2002_eva_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
	file_name_param = 'Couzin2002_eva_module_parameters';
	[~,str_core] = get_multi_core_value();
	fun_params = str2func([file_name_param,str_core]);
end
fun_params();

%
number = size(states,2);
c = states(1:3,:);
V = states(4:6,:);
V = V./(vecnorm(V) + 1e-8);
%
p_group = 1/number * norm(sum(V,2));
c_group = 1/number * sum(c,2);
temp = cross(c - c_group,V);
temp = temp./sqrt(sum(temp.^2));
temp_error = find(isnan(temp(1,:)));
if ~isempty(temp_error)
    temp(:,temp_error) = [0;0;0];
end
m_group_vector = 1/number*sum(temp,2);
m_group = norm(m_group_vector);

values = [p_group;m_group];




end