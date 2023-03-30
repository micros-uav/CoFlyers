function [flag_close_range,xrange,yrange,zrange,cylinder_radius,cylinder_rows_number,...
    cylinder_cols_number,cylinder_delta_x,cylinder_delta_y,cylinder_offset_x,...
    cylinder_offset_y,cylinder_disc_num,grid_delta_x,grid_delta_y] = map_module_parameters_deal(parameters_map)
%ENVIRONMENT_MODEL_PARAMETERS_DEAL 

myCount = 1;
flag_close_range = parameters_map(myCount:myCount+1); myCount = myCount + 1;
xrange = parameters_map(myCount:myCount+1); myCount = myCount + 2;
yrange = parameters_map(myCount:myCount+1); myCount = myCount + 2;
zrange = parameters_map(myCount:myCount+1); myCount = myCount + 2;
cylinder_radius      = parameters_map(myCount); myCount = myCount + 1;
cylinder_rows_number = parameters_map(myCount); myCount = myCount + 1;
cylinder_cols_number = parameters_map(myCount); myCount = myCount + 1;
cylinder_delta_x     = parameters_map(myCount); myCount = myCount + 1;
cylinder_delta_y     = parameters_map(myCount); myCount = myCount + 1;
cylinder_offset_x    = parameters_map(myCount); myCount = myCount + 1;
cylinder_offset_y    = parameters_map(myCount); myCount = myCount + 1;
cylinder_disc_num    = parameters_map(myCount); myCount = myCount + 1;
grid_delta_x    = parameters_map(myCount); myCount = myCount + 1;
grid_delta_y    = parameters_map(myCount); myCount = myCount + 1;
end

