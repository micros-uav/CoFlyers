function [x_range, y_range, z_range] = map_module_get_range(map3d_struct)
%MAP_MODULE_GET_RANGE Summary of this function goes here
%   Detailed explanation goes here
if ~isempty(map3d_struct)
    ind_range = find(map3d_struct(15,:) == -2);
    if ~isempty(ind_range)
%         x_range = map3d_struct(1,ind_range(1:2))-map3d_struct(7,ind_range(1:2))/2.*[-1,1];
%         y_range = map3d_struct(2,ind_range(3:4))-map3d_struct(8,ind_range(3:4))/2.*[-1,1];
%         z_range = map3d_struct(3,ind_range(5:6))-map3d_struct(9,ind_range(5:6))/2.*[-1,1];
        x_range = [min(map3d_struct(1,ind_range)),max(map3d_struct(1,ind_range))];
        y_range = [min(map3d_struct(2,ind_range)),max(map3d_struct(2,ind_range))];
        z_range = [min(map3d_struct(3,ind_range)),max(map3d_struct(3,ind_range))];
    else
        x_range = [];
        y_range = [];
        z_range = [];
    end
    if x_range(2) - x_range(1) ==0
        x_range = [0,1];
    end
    if y_range(2) - y_range(1) ==0
        y_range = [0,1];
    end
    if z_range(2) - z_range(1) ==0
        z_range = [0,1];
    end
else
        x_range = [0,1];
        y_range = [0,1];
        z_range = [0,1];
end

end