function face_s = generate_faces_from_stl(stl_name, position, rotation, scale)
%GENERATE_FACES_FROM_STL Summary of this function goes here
%   Detailed explanation goes here

TR = stlread(stl_name);
face_s= [TR.Points(TR.ConnectivityList(:,1),:),TR.Points(TR.ConnectivityList(:,2),:),TR.Points(TR.ConnectivityList(:,3),:)]';

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

x_mean = mean(face_s(1:3:9,:),'all');
y_mean = mean(face_s(2:3:9,:),'all');
z_mean = mean(face_s(3:3:9,:),'all');
p_mean = [x_mean;y_mean;z_mean];

face_s(1:3,:) = R*((face_s(1:3,:) - p_mean).*scale)  + p_mean + position;
face_s(4:6,:) = R*((face_s(4:6,:) - p_mean).*scale)  + p_mean + position;
face_s(7:9,:) = R*((face_s(7:9,:) - p_mean).*scale)  + p_mean + position;

end

