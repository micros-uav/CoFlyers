function draw_Lidar(position,angles,ranges,range_max,range_min,dimension)
%VISUALIZATION_MODULE_DRAW_LIDAR 

position2D = position(1:2);
num_angles = length(angles);

ranges(ranges == inf) = range_max;
x1 = ones(1,num_angles)*position2D(1) + range_min * cos(angles);
y1 = ones(1,num_angles)*position2D(2) + range_min * sin(angles);
x2 = ones(1,num_angles)*position2D(1) + ranges .* cos(angles);
y2 = ones(1,num_angles)*position2D(2) + ranges .* sin(angles);

if dimension == 3
    X = [x1(1:end-1);x1(2:end);x2(2:end);x2(1:end-1)];
    Y = [y1(1:end-1);y1(2:end);y2(2:end);y2(1:end-1)];
    Z = zeros(size(X)) + position(3);
%     patch(X,Y,Z,'blue','EdgeColor','none','FaceAlpha',0.5);
    patch(X,Y,Z,'blue','LineWidth',0.1,'FaceAlpha',0.5);
else
    X = [x1(1:end-1);x1(2:end);x2(2:end);x2(1:end-1)];
    Y = [y1(1:end-1);y1(2:end);y2(2:end);y2(1:end-1)];
    patch(X,Y,'blue','LineWidth',0.1,'FaceAlpha',0.5);
end

end

