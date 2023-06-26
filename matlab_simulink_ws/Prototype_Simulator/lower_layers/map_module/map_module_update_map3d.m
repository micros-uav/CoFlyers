function [map3d_faces, map3d_struct] = map_module_update_map3d(t, dt, map3d_faces_in,map3d_struct_in)
%MAP_MODULE_UPDATE_MAP3D Summary of this function goes here
%   Detailed explanation goes here
map3d_faces = map3d_faces_in;
map3d_struct = map3d_struct_in;
if isempty(map3d_struct)
    return
end

ind_dynamic = find(map3d_struct(14,:) == 0);
for i = 1:length(ind_dynamic)
    % Update
    ind_now = ind_dynamic(i);
    id = map3d_struct(15,ind_now);
    model_struct = map3d_struct(1:13,ind_now);
    model_struct = update_map3d_user(t,dt,model_struct,id);
    map3d_struct(1:13,ind_now) = model_struct;
    % Change map
    ind_1 = map3d_struct(end-1,ind_now);
    ind_2 = map3d_struct(end,ind_now);
    face_s_n_s = map3d_faces(1:12,ind_1:ind_2);
    position = map3d_struct(1:3,ind_now) - map3d_struct_in(1:3,ind_now);
    R = e2r(map3d_struct(4:6,ind_now))\e2r(map3d_struct_in(4:6,ind_now));
    scale = map3d_struct(7:9,ind_now)./map3d_struct(7:9,ind_now);
    x_mean = mean(face_s_n_s(1:3:9,:),'all');
    y_mean = mean(face_s_n_s(2:3:9,:),'all');
    z_mean = mean(face_s_n_s(3:3:9,:),'all');
    p_mean = [x_mean;y_mean;z_mean];
    face_s_n_s(1:3,:) = R*((face_s_n_s(1:3,:) - p_mean).*scale)  + p_mean + position;
    face_s_n_s(4:6,:) = R*((face_s_n_s(4:6,:) - p_mean).*scale)  + p_mean + position;
    face_s_n_s(7:9,:) = R*((face_s_n_s(7:9,:) - p_mean).*scale)  + p_mean + position;
    face_s_n_s(10:12,:) = R*face_s_n_s(10:12,:);
    map3d_faces(1:12,ind_1:ind_2) = face_s_n_s;
end

end

function R = e2r(rotation)
t1 = rotation(1);
t2 = rotation(2);
t3 = rotation(3);
r_z = [cosd(t3),-sind(t3),0
    sind(t3),cosd(t3),0
    0,0,1];
r_x = [1,0,0
    0,cosd(t1),-sind(t1)
    0,sind(t1),cosd(t1)];
r_y = [cosd(t2),0,sind(t2)
    0,1,0
    -sind(t2),0,cosd(t2)];
R = r_y*r_x*r_z;
end