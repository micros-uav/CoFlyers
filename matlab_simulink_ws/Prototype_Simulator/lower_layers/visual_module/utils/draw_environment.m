function  draw_environment(my_axes, map3d_faces, map3d_struct,model_stls, dim)
% DRAW_ENVIRONMENT 
%  EXAMPLE:
%     [map3d_faces, map3d_struct, model_stls] = generate_map3d();
%     figure;axis equal;xlabel('x');ylabel('y');hold on;my_axes = gca;
%     draw_environment(my_axes, map3d_faces, map3d_struct,model_stls, 3);

if isempty(map3d_struct)
    return;
end

% For simulink
if isempty(model_stls) && exist("model_stls.mat","file")
    model_stls = importdata("model_stls.mat");
end

%
obj = findobj(my_axes.Children,'Tag','model_1');
flag_init = isempty(obj);
if flag_init
    %====Initialize and label the image object of bodies=====%
    for ind_now = 1:size(map3d_struct,2)
        stl_name = model_stls(ind_now);
        color = map3d_struct(10:12,ind_now);
        alpha = map3d_struct(13,ind_now);
        position = map3d_struct(1:3,ind_now);
        rotation = map3d_struct(4:6,ind_now);
        scale = map3d_struct(7:9,ind_now);
        
        static = map3d_struct(14,ind_now);
        if static
            temp = strsplit(stl_name,'_rough');
            stl_name = strcat(temp{:});
        end

        face_s = generate_faces_from_stl(stl_name, position, rotation, scale);

        if dim==3
            p = patch(my_axes, 'XData',face_s(1:3:9,:),'YData',face_s(2:3:9,:),'ZData',face_s(3:3:9,:),'FaceColor',color,'FaceAlpha',alpha,'LineStyle','none');
%             if ~static
%                 ind_1 = map3d_struct(end-1,ind_now);
%                 ind_2 = map3d_struct(end,ind_now);
%                 q = quiver3(map3d_faces(1,ind_1:ind_2),map3d_faces(2,ind_1:ind_2),map3d_faces(3,ind_1:ind_2),...
%                     map3d_faces(10,ind_1:ind_2),map3d_faces(11,ind_1:ind_2),map3d_faces(12,ind_1:ind_2));
%                 q.Tag = strcat("mmodel_",num2str(ind_now));
%             end
        else
            p = patch(my_axes, 'XData',face_s(1:3:9,:),'YData',face_s(2:3:9,:),'FaceColor',color,'FaceAlpha',alpha,'LineStyle','none');
        end
        p.Tag = strcat("model_",num2str(ind_now));
    end
else

    %====Update data====%
    ind_dynamic = find(map3d_struct(14,:) == 0);
    for i = 1:length(ind_dynamic)
        ind_now = ind_dynamic(i);
        color = map3d_struct(10:12,ind_now);
        alpha = map3d_struct(13,ind_now);
        ind_1 = map3d_struct(end-1,ind_now);
        ind_2 = map3d_struct(end,ind_now);
        obj = findobj(my_axes.Children,'Tag',strcat("model_",num2str(ind_now)));
        if dim==3
            set(obj,'XData',map3d_faces(1:3:9,ind_1:ind_2), ...
                'YData',map3d_faces(2:3:9,ind_1:ind_2), ...
                'ZData',map3d_faces(3:3:9,ind_1:ind_2),'FaceColor',color,'FaceAlpha',alpha,'LineStyle','none');

%             obj = findobj(my_axes.Children,'Tag',strcat("mmodel_",num2str(ind_now)));
%             ind_1 = map3d_struct(end-1,ind_now);
%             ind_2 = map3d_struct(end,ind_now);
%             set(obj,'XData',map3d_faces(1,ind_1:ind_2),'YData',map3d_faces(2,ind_1:ind_2),'ZData',map3d_faces(3,ind_1:ind_2),...
%                 'UData',map3d_faces(10,ind_1:ind_2),'VData',map3d_faces(11,ind_1:ind_2),'WData',map3d_faces(12,ind_1:ind_2));
        else
            set(obj,'XData',map3d_faces(1:3:9,ind_1:ind_2), ...
                'YData',map3d_faces(2:3:9,ind_1:ind_2), ...
                'FaceColor',color,'FaceAlpha',alpha,'LineStyle','none');
        end
    end
end

end

