function [data] = map_module_get_params_for_visual(parameters_map)
%MAP_MODULE_GET_PARAMS_FOR_VISUAL Summary of this function goes here
%   Detailed explanation goes here


[~,x_range,y_range,z_range] = map_module_parameters_deal(parameters_map);
data = [x_range;y_range;z_range];

end

