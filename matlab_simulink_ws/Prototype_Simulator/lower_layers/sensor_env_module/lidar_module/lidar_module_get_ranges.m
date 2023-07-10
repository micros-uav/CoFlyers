function [range_s, psi_s, phi_s] = lidar_module_get_ranges(pos_agent, att_agent, map3d_faces)
%LIDAR_MODULE_GET_RANGES Summary of this function goes here
%   Detailed explanation goes here

file_name_param = 'lidar_module_parameters';
[~,str_core] = get_multi_core_value();
fun_params = str2func([file_name_param, str_core]);

[resolution,...
r_sense_min,...
r_sense_max,...
phi_range,...
psi_range] = fun_params();

[range_s,psi_s,phi_s] = get_lidar_from_map3d_faces(pos_agent, att_agent, map3d_faces, resolution, r_sense_min,  r_sense_max, phi_range, psi_range);

end

