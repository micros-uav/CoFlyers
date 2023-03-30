function parameters_map = map_module_parameters(number, flag_alg, p_bp)
%ENVIRONMENT_MODEL_PARAMETERS 

xrange =[-5,5];
yrange =[-5,5];
zrange =[0,3];

if number > 12
    xrange = xrange * (number/12)^(1/3);
    yrange = yrange * (number/12)^(1/3);
    zrange = zrange * (number/12)^(1/3);
end

cylinder_radius      = 0.3;
cylinder_rows_number = 0;
cylinder_cols_number = 0;
cylinder_delta_x     = 1.3;
cylinder_delta_y     = 1.3; 
cylinder_offset_x    = 0;
cylinder_offset_y    = 0; 
cylinder_disc_num    = 15;

grid_delta_x = 0.01;
grid_delta_y = 0.01;

flag_close_range = false; % when false,  the boundary of xrange and yrange is considered as walls 

if flag_alg == 2
    flag_close_range = true;
    cylinder_rows_number = 0;
    cylinder_cols_number = 0;
end

parameters_map = [
    flag_close_range
    xrange(:);
    yrange(:);
    zrange(:);
    cylinder_radius
    cylinder_rows_number
    cylinder_cols_number
    cylinder_delta_x
    cylinder_delta_y
    cylinder_offset_x
    cylinder_offset_y
    cylinder_disc_num
    grid_delta_x
    grid_delta_y];

% Modify parameters for batch processing
if ~isempty(p_bp)
    for k = 1:size(p_bp,2)
        parameters_map(p_bp(1,k)) = p_bp(2,k);
    end
end

end

