function draw_Lidar(my_axes, position, attitude, angles, ranges, range_max, range_min, dimension)
%VISUALIZATION_MODULE_DRAW_LIDAR

num_angles = length(angles);

ranges(ranges == inf) = range_max;
x1 = range_min * cos(angles);
y1 = range_min * sin(angles);
z1 = zeros(size(x1));
x2 = ranges .* cos(angles);
y2 = ranges .* sin(angles);
z2 = zeros(size(x2));
if dimension == 3
    t1 = attitude(1);
    t2 = attitude(2);
    t3 = attitude(3);
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
    temp = [x1;y1;z1];
    x1 = R(1,:)*temp;
    y1 = R(2,:)*temp;
    z1 = R(3,:)*temp;
    temp = [x2;y2;z2];
    x2 = R(1,:)*temp;
    y2 = R(2,:)*temp;
    z2 = R(3,:)*temp;
else
    R = [];
end

x1 = position(1) + x1;
y1 = position(2) + y1;
z1 = position(3) + z1;
x2 = position(1) + x2;
y2 = position(2) + y2;
z2 = position(3) + z2;



obj = findobj(my_axes.Children,'Tag','lidar');
flag_init = isempty(obj);
if flag_init
    %====Initialize and label the image object%
    X = [x1(1:end-1);x1(2:end);x2(2:end);x2(1:end-1)];
    Y = [y1(1:end-1);y1(2:end);y2(2:end);y2(1:end-1)];
    if dimension == 3
        Z = [z1(1:end-1);z1(2:end);z2(2:end);z2(1:end-1)];
        p = patch(my_axes, X,Y,Z,'blue','LineStyle','none','FaceAlpha',0.5);
    else
        p = patch(my_axes, X,Y,'blue','LineStyle','none','FaceAlpha',0.5);
    end
    p.Tag = "lidar";
else
    obj = findobj(my_axes.Children,'Tag',"lidar");
    X = [x1(1:end-1);x1(2:end);x2(2:end);x2(1:end-1)];
    Y = [y1(1:end-1);y1(2:end);y2(2:end);y2(1:end-1)];
    if dimension == 3
        Z = [z1(1:end-1);z1(2:end);z2(2:end);z2(1:end-1)];
        set(obj,'XData',X,'YData',Y,'ZData',Z);
    else
        set(obj,'XData',X,'YData',Y);
    end

end

end

