function ranges = Lidar_module_get_ranges(position2D,yaw,map_lines,parameters_Lidar)
%LIDAR_MODULE_GET_RANGES 此处显示有关此函数的摘要
%   此处显示详细说明

[~, ~, ~, ~, ~,...
    range_min, range_max] = Lidar_module_parameters_deal(parameters_Lidar);

angles = Lidar_module_get_angles(parameters_Lidar);
angles = angles + yaw;

num_angles = length(angles);
num_lines  = size(map_lines,2);
x1 = ones(1,num_angles)*position2D(1) + range_min * cos(angles);
y1 = ones(1,num_angles)*position2D(2) + range_min * sin(angles);
x2 = ones(1,num_angles)*position2D(1) + range_max * cos(angles);
y2 = ones(1,num_angles)*position2D(2) + range_max * sin(angles);

x3 = map_lines(1,:)'; 
y3 = map_lines(2,:)'; 
x4 = map_lines(3,:)';
y4 = map_lines(4,:)';

%%%  Get the intersection of the per two lines 

temp1 = ((x3-x4) .* (y1-y2) - (x1-x2) .* (y3-y4));
x0 = ((x3-x4) .* (x2.*y1 - x1.*y2) - (x1-x2) .* (x4.*y3 - x3.*y4)) ./ temp1;
y0 = ((y3-y4) .* (y2.*x1 - y1.*x2) - (y1-y2) .* (y4.*x3 - y3.*x4)) ./ ((y3-y4) .* (x1-x2) - (y1-y2) .* (x3-x4));

%%%  Search for points on the line segments

[x1_s,x2_s] = min_swap(x1,x2);
[y1_s,y2_s] = min_swap(y1,y2);
[x3_s,x4_s] = min_swap(x3,x4);
[y3_s,y4_s] = min_swap(y3,y4);

tolerance = 1e-10; % Eliminate truncation errors
x1_s = x1_s - tolerance;
x2_s = x2_s + tolerance;
y1_s = y1_s - tolerance;
y2_s = y2_s + tolerance;
x3_s = x3_s - tolerance;
x4_s = x4_s + tolerance;
y3_s = y3_s - tolerance;
y4_s = y4_s + tolerance;

% x0 = round(x0,10); % Eliminate truncation errors
% y0 = round(y0,10);

onlineSegment = (x0 >= x1_s & x0 <= x2_s & y0 >= y1_s & y0 <= y2_s) &...
    (x0 >= x3_s & x0 <= x4_s & y0 >= y3_s & y0 <= y4_s);

ind = find(onlineSegment);

x0_online = x0(ind);
y0_online = y0(ind);

% [~,cols] = ind2sub(size(onlineSegment),ind);
cols = ceil(ind/num_lines);

%%% Get Distances
dis = sqrt((x0_online - position2D(1)).^2 + (y0_online - position2D(2)).^2);

dis_sort_by_col = sortrows([cols,dis]);
dis = dis_sort_by_col(:,2);
unique_cols = cols - circshift(cols,1); unique_cols(1) = 1;
unique_cols = logical(unique_cols);
cols_select = cols(unique_cols);
ranges = inf(1,num_angles);
ranges(cols_select) = dis(unique_cols);

function [x1_s,x2_s] = min_swap(x1,x2)
x1_s = x1;
x2_s = x2;
jud = x1>x2;
x1_s(jud) = x2(jud);
x2_s(jud) = x1(jud);

end

end

