function [terrain, terrain_params] = get_terrain(str_png, position, scale)
%GET_TERRAIN 


%%% Check input %%%
if size(position,2) ~=1
    position = position';
end
if size(scale,2) ~=1
    scale = scale';
end
if isempty(str_png)||isempty(position)||isempty(scale)
    terrain = [];
    terrain_params = [];
    return;
end
%%% %%%
terrain = double(imread(str_png))-32768;
terrain_params = [position,scale];
terrain = terrain*terrain_params(3,2)+terrain_params(3,1);

end

