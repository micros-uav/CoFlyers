function  draw_environment(my_axes, map3d_faces, map3d_struct, model_stls, terrain, terrain_params, dim, cmap, clear_model)
% DRAW_ENVIRONMENT 
%  EXAMPLE:
%     [map3d_faces, map3d_struct, model_stls] = generate_map3d();
%     figure;axis equal;xlabel('x');ylabel('y');hold on;my_axes = gca;
%     draw_environment(my_axes, map3d_faces, map3d_struct,model_stls, 3);

if isempty(map3d_struct)
    return;
end

if nargin<8
    cmap = sky;
end
if nargin<9
    clear_model = false;
end

%
if clear_model
    % obj_s = findobj(my_axes.Children,'-regexp',"Tag","model_\w");
    % if ~isempty(obj_s)
        delete(findobj(my_axes.Children,'-regexp',"Tag","model_\w"));
    % end
end

obj = findobj(my_axes.Children,'Tag','model_1');
flag_init = isempty(obj);
if flag_init
    %================Draw terrain===============%
    if ~isempty(terrain)
        [h,w] = size(terrain);
        [xx,yy] = meshgrid(1:w,1:h);
        xx = xx*terrain_params(1,2)+ terrain_params(1,1);
        yy = yy*terrain_params(2,2)+ terrain_params(2,1);
        h_min = min(terrain,[],"all");
        h_max = max(terrain,[],"all");
        num_cmap = size(cmap,1);
        cc = reshape(cmap(floor((terrain-h_min)/(h_max-h_min)*(num_cmap-1))+1,:),[size(terrain),3]);
        if dim==3
            ter = mesh(my_axes, xx, yy, terrain, cc);
        else
            ter = imagesc(my_axes, xx(1,:),yy(:,1),cc);
        end
        ter.Tag = "terrain";
    end
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
        else
            set(obj,'XData',map3d_faces(1:3:9,ind_1:ind_2), ...
                'YData',map3d_faces(2:3:9,ind_1:ind_2), ...
                'FaceColor',color,'FaceAlpha',alpha,'LineStyle','none');
        end
    end
end

end

