function in_s = map_module_out_of_map(position3d_s, map3d_struct)
%MAP_MODULE_OUT_OF_MAP Summary of this function goes here
%   Detailed explanation goes here


if isempty(map3d_struct)
    in_s = true(1,size(position3d_s,2));
    return
end

ind_range = find(map3d_struct(15,:) == -2);
if ~isempty(ind_range)
    position_range_s = map3d_struct(1:3,ind_range);
    x_min = min(position_range_s(1,:));
    x_max = max(position_range_s(1,:));
    y_min = min(position_range_s(2,:));
    y_max = max(position_range_s(2,:));
    z_min = min(position_range_s(3,:));
    z_max = max(position_range_s(3,:));
end

in_s = position3d_s(1,:) < x_min | position3d_s(1,:) > x_max | position3d_s(2,:) < y_min |...
    position3d_s(2,:) > y_max | position3d_s(3,:) < z_min | position3d_s(3,:) > z_max;
end