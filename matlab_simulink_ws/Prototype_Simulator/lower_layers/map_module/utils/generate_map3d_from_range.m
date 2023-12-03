function [map3d_struct_0, model_stls] = generate_map3d_from_range(x_range,x_color,x_alpha,activate_x,...
    y_range,y_color,y_alpha,activate_y,...
    z_range,z_color,z_alpha,activate_z)
%GENERATE_MAP3D_FROM_RANGE 
%  
x_id = -2;
y_id = -2;
z_id = -2;

x_static = 1;
y_static = 1;
z_static = 1;

map3d_struct_0 = [];
model_stls = [];

if activate_x
    % x range1
    stl_name = "cube.stl";
    scale = [(x_range(2)-x_range(1))/100;
        y_range(2)-y_range(1);
        z_range(2)-z_range(1)];
    position = [x_range(1) - (x_range(2)-x_range(1))/200;
        mean(y_range);
        mean(z_range)];
    rotation = [0;0;0];
    map3d_struct_0 = [map3d_struct_0,[position(:);rotation(:);scale(:);x_color(:);x_alpha(:);x_static(:);x_id(:)]];
    model_stls = [model_stls, stl_name];
    % x range2
    stl_name = "cube.stl";
    scale = [(x_range(2)-x_range(1))/100;
        y_range(2)-y_range(1);
        z_range(2)-z_range(1)];
    position = [x_range(2) + (x_range(2)-x_range(1))/200;
        mean(y_range);
        mean(z_range)];
    rotation = [0;0;0];
    map3d_struct_0 = [map3d_struct_0,[position(:);rotation(:);scale(:);x_color(:);x_alpha(:);x_static(:);x_id(:)]];
    model_stls = [model_stls, stl_name];
end
if activate_y
    % y range1

    stl_name = "cube.stl";
    scale = [x_range(2)-x_range(1);
        (y_range(2)-y_range(1))/100;
        z_range(2)-z_range(1)];
    position = [mean(x_range);
        y_range(1) - (y_range(2)-y_range(1))/200;
        mean(z_range)];
    rotation = [0;0;0];
    map3d_struct_0 = [map3d_struct_0,[position(:);rotation(:);scale(:);y_color(:);y_alpha(:);y_static(:);y_id(:)]];
    model_stls = [model_stls, stl_name];
    % y range2
    stl_name = "cube.stl";
    scale = [x_range(2)-x_range(1);
        (y_range(2)-y_range(1))/100;
        z_range(2)-z_range(1)];
    position = [mean(x_range);
        y_range(2) + (y_range(2)-y_range(1))/200;
        mean(z_range)];
    rotation = [0;0;0];
    map3d_struct_0 = [map3d_struct_0,[position(:);rotation(:);scale(:);y_color(:);y_alpha(:);y_static(:);y_id(:)]];
    model_stls = [model_stls, stl_name];
end
if activate_z
    % z range1
    stl_name = "cube.stl";
    scale = [x_range(2)-x_range(1);
        y_range(2)-y_range(1);
        (z_range(2)-z_range(1))/100];
    position = [mean(x_range);
        mean(y_range);
        z_range(1) - (z_range(2)-z_range(1))/200];
    rotation = [0;0;0];
    map3d_struct_0 = [map3d_struct_0,[position(:);rotation(:);scale(:);z_color(:);z_alpha(:);z_static(:);z_id(:)]];
    model_stls = [model_stls, stl_name];

    % z range2
    stl_name = "cube.stl";
    scale = [x_range(2)-x_range(1);
        y_range(2)-y_range(1);
        (z_range(2)-z_range(1))/100];
    position = [mean(x_range);
        mean(y_range);
        z_range(2) + (z_range(2)-z_range(1))/200];
    rotation = [0;0;0];
    map3d_struct_0 = [map3d_struct_0,[position(:);rotation(:);scale(:);z_color(:);z_alpha(:);z_static(:);z_id(:)]];
    model_stls = [model_stls, stl_name];
end

end

