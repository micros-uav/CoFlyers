% function [map3d_faces, map3d_struct] = visual_module_buttondown(t, axis_1, map3d_faces_in,map3d_struct_in)
function [map3d_faces,map3d_struct] = visual_module_buttondown(t, axis_1, map3d_faces_in,map3d_struct_in)
%VISUAL_MODULE_BUTTONDOWN

persistent point_mouse

map3d_faces = map3d_faces_in;
map3d_struct = map3d_struct_in;

if isempty(axis_1.ButtonDownFcn) || t==0
    set(axis_1,'ButtonDownFcn',@(src,event)get_mouse_position(src,event,axis_1),'HitTest','on')
    point_mouse = [];
end

ind_dynamic = find(map3d_struct(14,:) == 0);
if ~isempty(ind_dynamic)
    %==========================

    ind_now = ind_dynamic(1);
    id = map3d_struct(15,ind_now);
    if id ==0
        model_struct = map3d_struct(1:13,ind_now);

        if isempty(point_mouse)
            point_mouse = zeros(2,1);
            point_mouse(1) = model_struct(1);
            point_mouse(2) = model_struct(2);
        end
        model_struct(1) = point_mouse(1);
        model_struct(2) = point_mouse(2);

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
%===========================

    function get_mouse_position(src,~,ax)
        point_mouse = ax.CurrentPoint(1,:);
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
end

