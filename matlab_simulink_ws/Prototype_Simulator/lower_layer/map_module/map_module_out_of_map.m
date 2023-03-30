function result = map_module_out_of_map(xs,ys,parameters_map,map_grid)
%MAP_MODULE_OUT_OF_MAP 


[~,xrange,yrange,~,~,~,~,~,~,~,~,~,grid_delta_x,grid_delta_y] = map_module_parameters_deal(parameters_map);

result = xs*0;
% out_of_area = xs<xrange(1) | xs>xrange(2) | ys<xrange(1) | ys>yrange(2);

% result(out_of_area) = true;

% xs_in_area = xs(~out_of_area);
% ys_in_area = ys(~out_of_area);

% cols_in_area_sub = floor((xs_in_area - xrange(1))/grid_delta_x) + 1;
% rows_in_area_sub = floor((ys_in_area - yrange(1))/grid_delta_y) + 1;

cols_in_map = floor((xs - xrange(1))/grid_delta_x) + 1;
rows_in_map = floor((ys - yrange(1))/grid_delta_y) + 1;

[h,w] = size(map_grid);

%%% 
out_of_area = cols_in_map < 1 | cols_in_map >w | rows_in_map < 1 |...
    rows_in_map > h;
no_out_of_area = ~out_of_area;

result(out_of_area) = true;
% result(~out_of_area) = map_grid(~out_of_area);
%%%
cols_in_area = cols_in_map(no_out_of_area);
if ~isempty(cols_in_area)
    rows_in_area = rows_in_map(no_out_of_area);

    ind = sub2ind([h,w],rows_in_area,cols_in_area);
    result(no_out_of_area) = map_grid(ind);
end

end

