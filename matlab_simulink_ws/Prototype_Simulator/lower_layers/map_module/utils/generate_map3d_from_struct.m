function [map3d_faces, map3d_struct, map3d_grid] = generate_map3d_from_struct(map3d_struct_0, model_stls)
%GENERATE_MAP3D_FROM_STRUCT Summary of this function goes here
%   Detailed explanation goes here
map3d_faces = []; 
map3d_grid = [];

num_rows = size(map3d_struct_0,1);
map3d_struct = zeros(size(map3d_struct_0)+[2,0]);
map3d_struct(1:num_rows,:) = map3d_struct_0;

ind_faces = 0;
for i = 1:size(map3d_struct_0,2)
    % Get faces from stl
    stl_name = model_stls(i);
    position = map3d_struct_0(1:3,i);
    rotation = map3d_struct_0(4:6,i);
    scale = map3d_struct_0(7:9,i);
    face_s = generate_faces_from_stl(stl_name, position, rotation, scale);
    AB = face_s(1:3,:) - face_s(4:6,:);
    CB = face_s(7:9,:) - face_s(4:6,:);
    p = (face_s(1:3,:) + face_s(4:6,:) + face_s(7:9,:))/3;
    n = cross(AB,CB);
    n = n./vecnorm(n);
    p = p + n*0.00001;
    in_s = in_model(p,face_s);
    n(:,in_s) = -n(:,in_s);
    % Store faces
    map3d_faces = [map3d_faces,[face_s;n]];
    % Store the indices of faces
    ind_faces = ind_faces + 1;
    map3d_struct(num_rows+1,i) = ind_faces;
    ind_faces = ind_faces + size(face_s,2) - 1;
    map3d_struct(num_rows+2,i) = ind_faces;
end

end