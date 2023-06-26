function [map_lines, ind] = get_map3d_profile(A,B,C,map_faces)
%GET_MAP3D_PROFILE Summary of this function goes here
%   Detailed explanation goes here
% Example:
%     map3d_faces = generate_map3d();
%     A = [5;-5;0]; B = [-5;5;3]; C = [5;5;3];
%     map3d_lines = get_map3d_profile(A,B,C,map3d_faces);
%     figure;patch(map3d_faces(1:3:9,:),map3d_faces(2:3:9,:),map3d_faces(3:3:9,:),'r','FaceAlpha',0.5);
%     axis equal;xlabel('x');ylabel('y');hold on;
%     plot3(map3d_lines([1,4],:),map3d_lines([2,5],:),map3d_lines([3,6],:),'LineWidth',2,'Color','g');
%     quiver3(map3d_lines(1,:),map3d_lines(2,:),map3d_lines(3,:),map3d_lines(7,:),map3d_lines(8,:),map3d_lines(9,:));
%     patch([A(1),B(1),C(1)],[A(2),B(2),C(2)],[A(3),B(3),C(3)],'k','FaceAlpha',0.5);


p1 = reshape([map_faces(1:3,:);map_faces(4:6,:);map_faces(7:9,:)],3,3*size(map_faces,2));
p2 = reshape([map_faces(4:6,:);map_faces(7:9,:);map_faces(1:3,:)],3,3*size(map_faces,2));
%===============Get intersect between lines and face ABC=================
% p = intersect_line_and_face(p1,p2,A,B,C);
n = cross(A-B,A-C);
n = n./vecnorm(n,2,1);

a = p2 - p1;
b = A - p1;

t1 = sum(n.*a,1);
t2 = sum(n.*b,1);

temp = t2./t1;
p = t2./t1.*a + p1;

p(:,(temp>1|isinf(temp)|temp<=0)) = nan;
%==========================================================
pp = reshape(p,9,size(p,2)/3);
ind = find(sum(isinf(pp)|isnan(pp))<6);
ppp = pp(:,ind);
temp = ~isnan(ppp);
% temp2 = sum(temp,1)~=6;
% temp(temp2) = 0;
% ind(temp2) = [];
map_lines_0 = reshape(ppp(temp),6,size(ppp,2));
normal_vecter = map_faces(10:12,ind);

normal_vecter = normal_vecter - sum(normal_vecter.*n,1).*n;
normal_vecter = normal_vecter./vecnorm(normal_vecter,2,1);

map_lines = [map_lines_0;normal_vecter];
end

