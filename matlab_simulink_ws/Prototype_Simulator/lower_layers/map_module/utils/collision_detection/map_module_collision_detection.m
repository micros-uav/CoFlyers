function in_s = map_module_collision_detection(point3d_s, map3d_faces, r_coll)
%MAP_MODULE_COLLISION_DETECTION Summary of this function goes here
%   Detailed explanation goes here

in_s = (sum(distance2faces(point3d_s, map3d_faces) < r_coll,2)>0)';

end