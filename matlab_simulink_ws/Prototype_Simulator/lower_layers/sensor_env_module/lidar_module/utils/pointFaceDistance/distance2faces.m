function distance_s = distance2faces(P_s_in, face_s_in)
%DISTANCE2FACES Summary of this function goes here
%   Detailed explanation goes here

num_p = size(P_s_in,2);
num_face = size(face_s_in,2);

P_s = repmat(P_s_in,[1,1,num_face]);
face_s = permute(repmat(face_s_in,[1,1,num_p]),[1,3,2]);

A = face_s(1:3,:,:);
B = face_s(4:6,:,:);
C = face_s(7:9,:,:);

b = B-A;
c = C-A;
d = P_s-A;

e1 = b;
e3 = cross(b,c);
e2 = cross(e3,e1);

e1 = e1./vecnorm(e1);
e2 = e2./vecnorm(e2);
e3 = e3./vecnorm(e3);

p1 = sum(d.*e1,1);
p2 = sum(d.*e2,1);
% p3 = dot(d, e3);

D_proj = p1.*e1+p2.*e2+A;

in_face_s = point_in_triangle(D_proj,A,B,C);

distance_v_s = abs(dot(d, e3));


d_1 = distance_point2line(P_s, A, B);
d_2 = distance_point2line(P_s, C, B);
d_3 = distance_point2line(P_s, C, A);
d_123 = min([d_1;d_2;d_3],[],1);

distance_s = zeros(size(in_face_s));
distance_s(in_face_s) = distance_v_s(in_face_s);
distance_s(~in_face_s) = d_123(~in_face_s);

distance_s = squeeze(distance_s);
end

