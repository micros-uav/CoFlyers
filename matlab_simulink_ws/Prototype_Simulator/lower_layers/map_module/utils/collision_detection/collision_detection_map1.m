function in_s = collision_detection_map1(point3d_s, map3d_faces, map3d_struct, r_coll)
%COLLISION_DETECTION_MAP Summary of this function goes here
%   Detailed explanation goes here

number = size(point3d_s,2);
in_s = false(1,number);

for i = 1:size(map3d_struct,2)
    ind = find(~in_s);
    ind_1 = map3d_struct(end-1,i);
    ind_2 = map3d_struct(end,i);
    [in_s_0, d_min_s] = in_model(point3d_s(:,ind),map3d_faces(:,ind_1:ind_2));
    in_s(ind) = in_s_0|(d_min_s<r_coll);
    if isempty(ind)
        break
    end
end

end