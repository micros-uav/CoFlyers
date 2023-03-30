function  draw_environment(my_axes_1, map_lines, dim, zrange)
%PLOT_MODEL_DRAW_ENVIRONMENT 
if isempty(map_lines)
    return;
end
% Proporties
boundary_identity = 0;
boundary_color = [0,0,0];
boundary_alpha = 0.2;
cylinder_identity = 1;
cylinder_color = [0.6,0.6,0.6];
cylinder_alpha = 1.0;
quadrilateral_identity = 2;
quadrilateral_color = [255,75,0]/255;
quadrilateral_alpha = 1.0;
polygonal_column_identity = 3;
polygonal_column_color = [236,176,31]/255;
polygonal_column_alpha = 1.0;

identities = [boundary_identity;cylinder_identity;quadrilateral_identity;polygonal_column_identity];
colors = [boundary_color;cylinder_color;quadrilateral_color;polygonal_column_color];
alphas = [boundary_alpha;cylinder_alpha;quadrilateral_alpha;polygonal_column_alpha];

[~,ind]=ismember(map_lines(7,:),identities);
ind(ind == 0) = 1;
map_colors = reshape(colors(ind,:),[length(ind),1,3]);
map_alphas = alphas(ind,:);

% Change the data format to fit the input of the 'patch/plot' function
x_s = map_lines([1,3],:);
y_s = map_lines([2,4],:);
if dim == 3
    x_ss = [x_s;flip(x_s)];
    y_ss = [y_s;flip(y_s)];
    z_ss = [y_s*0 + zrange(1);flip(y_s)*0+zrange(2)];
    environments = cat(3,x_ss,y_ss,z_ss);
elseif dim == 2
    x_ss = [x_s;x_s(1,:)*nan];
    y_ss = [y_s;y_s(1,:)*nan];
    environments = cat(3,x_ss(:),y_ss(:));
end

obj = findobj(my_axes_1.Children,'Tag','Env');
flag_init = isempty(obj);
if flag_init
    %====Initialize and label the image object of bodies=====%
    if dim==3
        p = patch(my_axes_1, environments(:,:,1),environments(:,:,2),environments(:,:,3),map_colors,...
            'FaceAlpha','flat','FaceVertexAlphaData',map_alphas*100,'AlphaDataMapping','direct');
    elseif dim == 2
        p = plot(my_axes_1, environments(:,:,1),environments(:,:,2),'k','LineWidth',2);
    end
    p.Tag = 'Env';
else
    %====Update data====%
    if dim==3
        set(obj,'XData',environments(:,:,1),'YData',environments(:,:,2),'ZData',environments(:,:,3));
    elseif dim ==2
        set(obj,'XData',environments(:,:,1),'YData',environments(:,:,2));
    end
end

end

