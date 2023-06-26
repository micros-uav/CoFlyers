function [map3d_struct_0, model_stls, ind_models] = read_map_param_struct(s)
%READ_MAP_PARAM_STRUCT Summary of this function goes here
%   Detailed explanation goes here

map3d_struct_0 = [];
model_stls = [];

if ~s.activate
    return
end

field_names = fieldnames(s);

%===================Get models===================%
ind_models = find(cellfun(@(x)~isempty(x),strfind(field_names,"model"))' &...
    cellfun(@(x)strlength(x)==6,field_names)');
for i = 1:length(ind_models)
    fn = field_names{ind_models(i)};

    %
    subfield_names = fieldnames(s.(fn));
    dim = unique(cellfun(@(x)size(s.(fn).(x),2),subfield_names)');

    if length(dim)>1
        dim(dim==1) = [];
        if length(dim)>1
            error(strcat("Dimension error in ",fn,"."));
        end
    end

    %
    stl_name = s.(fn).stl;
    scale = s.(fn).scale;
    position = s.(fn).position;
    rotation = s.(fn).rotation;
    color = s.(fn).color;
    alpha = s.(fn).alpha;
    static = s.(fn).static;
    %
    stl_name = repmat(stl_name,1,dim/size(stl_name,2));
    scale = repmat(scale,1,dim/size(scale,2));
    position = repmat(position,1,dim/size(position,2));
    rotation = repmat(rotation,1,dim/size(rotation,2));
    color = repmat(color,1,dim/size(color,2));
    alpha = repmat(alpha,1,dim/size(alpha,2));
    static = repmat(static,1,dim/size(static,2));
    id_s = zeros(1,dim) - 1;
    if sum(1-static) > 0
        if ~isfield(s.(fn),"id")
            error(strcat(fn," has dynamic obstacles, therefore ID needs to be provided."));
        else
            id_s = s.(fn).id;
            if size(id_s,2) ~=dim
                error(strcat("Different models need to provide different IDs in ",fn));
            end
        end
    end

    map3d_struct_0 = [map3d_struct_0,[position;rotation;scale;color;alpha;static;id_s]];
    model_stls = [model_stls,strtrim(string(stl_name))];
end



%=======================

x_range = s.x_range;
x_color = s.x_color;
x_alpha = s.x_alpha;
x_static = 1;
x_id = -2;

y_range = s.y_range;
y_color = s.y_color;
y_alpha = s.y_alpha;
y_static = 1;
y_id = -2;

z_range = s.z_range;
z_color = s.z_color;
z_alpha = s.z_alpha;
z_static = 1;
z_id = -2;

if isempty(x_range)
    x_range = [0,1];
    ignore_x_range = true;
else
    ignore_x_range = false;
end
if isempty(y_range)
    y_range = [0,1];
    ignore_y_range = true;
else
    ignore_y_range = false;
end
if isempty(z_range)
    z_range = [0,1];
    ignore_z_range = true;
else
    ignore_z_range = false;
end

%=====================range2cube====================%
if ~ignore_x_range
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
    model_stls = [model_stls, string(stl_name)];
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
    model_stls = [model_stls, string(stl_name)];
end
if ~ignore_y_range
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
    model_stls = [model_stls, string(stl_name)];
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
    model_stls = [model_stls, string(stl_name)];
end
if ~ignore_z_range
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
    model_stls = [model_stls, string(stl_name)];

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
    model_stls = [model_stls, string(stl_name)];
end
end

