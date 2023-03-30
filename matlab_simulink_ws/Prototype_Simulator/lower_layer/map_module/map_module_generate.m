function [map_lines,map_grid] = map_module_generate(parameters_map)
%ENVIRONMENT_MODEL_GENERATE 2D map
% map_lines: all obstacles are separated into segments and each column
% represents a line segment obstacle. The elements in each column are
% sequentially the starting point, the ending point, and the unit vector
% pointing to the interior of the obstacle, as well as the type of
% the obstacle.


% Get parameters
[flag_close_range,xrange,yrange,zrange,cylinder_radius,cylinder_rows_number,...
    cylinder_cols_number,cylinder_delta_x,cylinder_delta_y,cylinder_offset_x,...
    cylinder_offset_y,cylinder_disc_num,grid_delta_x,grid_delta_y] =...
    map_module_parameters_deal(parameters_map);

% The identity of line segments
boundary_identity = 0;   %
cylinder_identity = 1;
quadrilaterals_identity = 2;
polygonal_column_identity = 3;
%=================================B


%==================================Cylinder================================
% [center_x;center_y;radius]
cylinders = zeros(3,cylinder_cols_number*cylinder_rows_number - floor(cylinder_cols_number/2));
myCount = 1;
for i = 1:cylinder_cols_number
    flag = mod(i-1,2);
    for j = 1:cylinder_rows_number-(flag)
        % Odd rows obstacles
        cylinders(:,myCount) = [cylinder_delta_x*(i-1)
            cylinder_delta_y*(j-1)+flag*cylinder_delta_y/2
            cylinder_radius];
        myCount = myCount +1;
    end
end
mean_xy = mean(cylinders(1:2,:),2);
cylinders(1:2,:) = cylinders(1:2,:) - mean_xy + [mean(xrange);mean(yrange)];
cylinders(1,:) = cylinders(1,:) + cylinder_offset_x;
cylinders(2,:) = cylinders(2,:) + cylinder_offset_y;
%=================================quadrilateral============================
quadrilaterals = [];

% quadrilaterals1 = [[0;-2],[0.5;-2.5],[1;-2],[0;-2]];
% quadrilaterals2 = [[-0.5;2],[0;1.5],[1;2],[0.5;2.5]];
% quadrilaterals = [quadrilaterals,quadrilaterals1(:)];
% quadrilaterals = [quadrilaterals,quadrilaterals2(:)];

%==============================polygonal column=============================
%  Each polygon column needs to be defined separately.
%  The first and last points are connected to close the shape.
polygonal_column = [];
% polygonal_column = [[0;0],[1;0],[2;1],[1;3],[1;2]];


%========================Boundary (polygonal column)=======================
boundary_poly = [];
% boundary_poly= [-2.0000 -3.5000
%     2.0000   -3.5000
%     2.0000   -0.9000
%    -0.6000   -0.9000
%    -0.6000    0.3000
%     2.0000    0.3000
%     2.0000    2.8000
%    -2.0000    2.8000]';
%%% 

%============= Convert to start and end points of line segments ===========
% arena lines
if flag_close_range
    ranges_lines_directions = [];
else
    ranges_lines = [xrange(1),yrange(1),xrange(1),yrange(2)
        xrange(1),yrange(2),xrange(2),yrange(2)
        xrange(2),yrange(2),xrange(2),yrange(1)
        xrange(2),yrange(1),xrange(1),yrange(1)]';
    ranges_lines_directions = [ranges_lines;
        -get_vertical_direciton(ranges_lines(1:2,:),ranges_lines(3:4,:),[mean(xrange);mean(yrange)]);
        repelem(boundary_identity,1,size(ranges_lines,2))];
end

% Cylinder lines
cylinders_lines_directions = get_cylinders_lines_directions(cylinders,cylinder_disc_num);

cylinders_lines_directions = [cylinders_lines_directions;
    repelem(cylinder_identity,1,size(cylinders_lines_directions,2))];

% Quadrilaterals lines
quadrilaterals_lines_directions = get_quadrilaterals_lines_directions(quadrilaterals);

quadrilaterals_lines_directions = [quadrilaterals_lines_directions;
    repelem(quadrilaterals_identity,1,size(quadrilaterals_lines_directions,2))];

% One polygonal column
if ~isempty(polygonal_column)
polygonal_column_lines_directions = get_polygonal_column_lines_directions(polygonal_column);
polygonal_column_lines_directions = [polygonal_column_lines_directions;
    repelem(polygonal_column_identity,1,size(polygonal_column_lines_directions,2))];
else
    polygonal_column_lines_directions = [];
end

% Boundary
if ~isempty(boundary_poly)
boundary_poly_lines_directions = get_polygonal_column_lines_directions(boundary_poly);
boundary_poly_lines_directions(5:6,:) = -boundary_poly_lines_directions(5:6,:);
boundary_poly_lines_directions = [boundary_poly_lines_directions;
    repelem(boundary_identity,1,size(boundary_poly_lines_directions,2))];
else
    boundary_poly_lines_directions = [];
end


% Combine
map_lines = [ranges_lines_directions,...
    cylinders_lines_directions,...
    quadrilaterals_lines_directions,...
    polygonal_column_lines_directions,...
    boundary_poly_lines_directions];

%===============================Grid Map===================================

x = xrange(1):grid_delta_x:xrange(2);
y = yrange(1):grid_delta_y:yrange(2);
[xx,yy] = meshgrid(x,y);
% Arena ranges
% map_grid = ~inpolygon(xx,yy,ranges_lines(1,:),ranges_lines(2,:));
map_grid = false(size(xx));
% Cylinders
phiCircle = linspace(-pi,pi,100);
for k = 1:size(cylinders,2)
    circleX = cylinders(3,k) * cos(phiCircle) + cylinders(1,k);
    circleY = cylinders(3,k) * sin(phiCircle) + cylinders(2,k);
    map_grid = map_grid | inpolygon(xx,yy,circleX,circleY);
end

% Quadrilaterals
for k = 1:size(quadrilaterals,2)
    qx = quadrilaterals(1:2:8,k);
    qy = quadrilaterals(2:2:8,k);
    map_grid = map_grid | inpolygon(xx,yy,qx,qy);
end

% Polygonal column
if ~isempty(polygonal_column)
    map_grid = map_grid | inpolygon(xx,yy,polygonal_column(1,:),polygonal_column(2,:));
end

% Boundary
if ~isempty(boundary_poly)
    map_grid = map_grid | (~inpolygon(xx,yy,boundary_poly(1,:),boundary_poly(2,:)));
end

%%% Functions
    function cylinders_lines_directions = get_cylinders_lines_directions(cylinders,cylinder_disc_num)
        cylinders_lines_directions = zeros(6,cylinder_disc_num*size(cylinders,2));
        phi = linspace(-pi,pi,cylinder_disc_num+1);
        phi(end) = [];
        for ci = 1:size(cylinders,2)
            center = cylinders(1:2,ci);
            radius = cylinders(3,ci);
            ps = radius * [cos(phi);sin(phi)] + center;
            cylinders_lines_directions(1:6,1+(ci-1)*cylinder_disc_num:ci*cylinder_disc_num) =...
                get_polygonal_column_lines_directions(ps);
        end
    end

    function quadrilaterals_lines_directions = get_quadrilaterals_lines_directions(quadrilaterals)
        num = size(quadrilaterals,2);
        quadrilaterals_lines_directions = zeros(6,num*4);
        for qi = 1:num
            quadrilateral = [quadrilaterals(1:2:end,qi)';quadrilaterals(2:2:end,qi)'];
            quadrilaterals_lines_directions(:,1+(qi-1)*4:qi*4) = ...
                get_polygonal_column_lines_directions(quadrilateral);
        end
    end

    function polygonal_column_lines_directions = get_polygonal_column_lines_directions(polygonal_column)
        lines = [polygonal_column;circshift(polygonal_column,1,2)];
        directions = get_vertical_direciton(lines(1:2,:),lines(3:4,:),[mean(lines(1,:));mean(lines(2,:))]);
        interior_points = [mean(lines([1,3],:)) + directions(1,:)*0.00001;
            mean(lines([2,4],:)) + directions(2,:)*0.00001];
        in_polygon = inpolygon(interior_points(1,:),interior_points(2,:),polygonal_column(1,:), ...
            polygonal_column(2,:));
        directions(:,in_polygon) = -directions(:,in_polygon);
        polygonal_column_lines_directions = [lines;-directions];
    end

% Obtain the normal unit vector of the line segment composed of the starting point and the ending point
    function v_normal = get_vertical_direciton(point_start,point_end,point_outer)
        %         num = size(point_start,2);
        v = point_end - point_start;
        v_norm = sqrt(sum(v.^2,1));
        temp = v_norm~=0;
        if sum(temp) > 0
            v(:,temp) = v(:,temp)./v_norm(temp);
        end
        v_normal = [cos(pi/2),sin(pi/2);-sin(pi/2),cos(pi/2)]*v;
        temp = dot(point_outer-point_start,v_normal,1) < 0;
        if sum(temp) > 0
            v_normal(:,temp) = -v_normal(:,temp);
        end
    end
%     function environment_grid = get_grid(environment_lines,delta_x,delta_y,xrange,yrange)
%         x = xrange(1):delta_x:xrange(2);
%         y = yrange(1):delta_y:yrange(2);
%         [xx,yy] = meshgrid(x,y);
%         environment_grid = xx*0;
%         for icc = 1:numel(environment_grid)
%             environment_grid(icc) = occupy_determine_by_map_lines(xx(icc)+1e-9,yy(icc)+1e-9,environment_lines);
%         end
% 
%     end
end

