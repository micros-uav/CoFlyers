function  stl_name_new = decrease_face(stl_name, scale)
%DECREASE_FACE Summary of this function goes here
%   Detailed explanation goes here
if scale >= 1 || scale <=0
    return
end
temp = strsplit(stl_name,'.');
if length(temp) == 1
    stl_name_prefix = stl_name;
    stl_name_suffix = "stl";
    stl_name = strcat(stl_name_prefix,'.',stl_name_suffix);
else
    stl_name_prefix = temp{1};
    stl_name_suffix = temp{2};
end
stl_name_new = strcat(stl_name_prefix,"_new.",stl_name_suffix);


%====================================%
TR = stlread(stl_name);
face_s = [TR.Points(TR.ConnectivityList(:,1),:),TR.Points(TR.ConnectivityList(:,2),:),TR.Points(TR.ConnectivityList(:,3),:)]';
figure;
hold on;
axis equal;
p = patch(face_s(1:3:end,:),face_s(2:3:end,:),face_s(3:3:end,:),'r');
reducepatch(p,scale);
%====================================%
points = p.Vertices;
faces_ind = p.Faces;
TR = triangulation(faces_ind,points);
% figure;
% trimesh(TR)
% axis equal
stlwrite(TR,stl_name_new,'text')

end

