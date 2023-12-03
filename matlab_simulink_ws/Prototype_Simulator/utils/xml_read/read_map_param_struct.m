function [map3d_struct_0, model_stls, ind_models] = read_map_param_struct(s)
%READ_MAP_PARAM_STRUCT Summary of this function goes here
%   Detailed explanation goes here

map3d_struct_0 = [];
model_stls = [];
ind_models = [];

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
    ind_no_stls = ~(cellfun(@(x)~isempty(x),strfind(subfield_names,"stl"))');
    % dim = unique(cellfun(@(x)size(s.(fn).(x),2),subfield_names)');
    dim = unique(cellfun(@(x)size(s.(fn).(x),2),subfield_names(ind_no_stls))');

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

y_range = s.y_range;
y_color = s.y_color;
y_alpha = s.y_alpha;

z_range = s.z_range;
z_color = s.z_color;
z_alpha = s.z_alpha;

if isempty(x_range)
    x_range = [0,1];
    activate_x = false;
else
    activate_x = true;
end
if isempty(y_range)
    y_range = [0,1];
    activate_y = false;
else
    activate_y = true;
end
if isempty(z_range)
    z_range = [0,1];
    activate_z = false;
else
    activate_z = true;
end

%=====================range2cube====================%

[map3d_struct_0_, model_stls_] = generate_map3d_from_range(x_range,x_color,x_alpha,activate_x,...
    y_range,y_color,y_alpha,activate_y,...
    z_range,z_color,z_alpha,activate_z);

map3d_struct_0 = [map3d_struct_0,map3d_struct_0_];
model_stls = [model_stls,model_stls_];

end

