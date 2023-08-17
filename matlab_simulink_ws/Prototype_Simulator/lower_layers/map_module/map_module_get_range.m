function [x_range, y_range, z_range] = map_module_get_range(map3d_struct)
%MAP_MODULE_GET_RANGE Summary of this function goes here
%   Detailed explanation goes here
if ~isempty(map3d_struct)
    x_range = [];
    y_range = [];
    z_range = [];
    ind_range_s = find(map3d_struct(15,:) == -2);
    if ~isempty(ind_range_s)
        ind_s_range_f = map3d_struct(end-1,ind_range_s);
        ind_e_range_f = map3d_struct(end,ind_range_s);
        ind = [];
        for i = 1:length(ind_s_range_f)
            ind_range_f = [ind, ind_s_range_f(i):ind_e_range_f(i)];
        end
        for i = 1:2:length(ind_range_s)
            ind_r_s_i = ind_range_s(i);
            for j = 1:3
                v_min = map3d_struct(j,ind_r_s_i);
                v_max = map3d_struct(j,ind_r_s_i+1);
                if v_min ~= v_max
                    v_spin = map3d_struct(6+j,ind_r_s_i)/2;
                    switch j
                        case 1
                            x_range = [v_min+v_spin,v_max-v_spin];
                        case 2
                            y_range = [v_min+v_spin,v_max-v_spin];
                        case 3
                            z_range = [v_min+v_spin,v_max-v_spin];
                    end
                    continue
                end
            end
        end
    end
    if isempty(x_range)
        x_range = [0,1];
    end
    if isempty(y_range)
        y_range = [0,1];
    end
    if isempty(z_range)
        z_range = [0,1];
    end
else
        x_range = [0,1];
        y_range = [0,1];
        z_range = [0,1];
end

end