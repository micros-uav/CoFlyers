function d = distance_point2line(pt, v1, v2)
%DISTANCE_POINT2LINE Summary of this function goes here
%   Detailed explanation goes here

a = v2 - v1;
b = pt - v1;
c = pt - v2;

temp1 = vecnorm(a,2,1);
d_1 = vecnorm(cross(a,b),2,1)./temp1;

no_in_line = dot(a,b) < 0 | dot(a,c) > 0;
d_2 = vecnorm(b,2,1);
d_3 = vecnorm(c,2,1);

d = zeros(size(no_in_line));
d(~no_in_line) = d_1(~no_in_line);
d(no_in_line) = min(d_2(no_in_line), d_3(no_in_line));

end