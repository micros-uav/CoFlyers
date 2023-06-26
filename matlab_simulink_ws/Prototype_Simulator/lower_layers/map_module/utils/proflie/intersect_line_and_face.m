function p = intersect_line_and_face(p1,p2,A,B,C)
%INTERSECT_LINE_AND_FACE Summary of this function goes here
%   Detailed explanation goes here

n = cross(A-B,A-C);
n = n./vecnorm(n,2,1);

a = p2 - p1;
b = A - p1;

t1 = sum(n.*a,1);
t2 = sum(n.*b,1);

temp = t2./t1;
p = t2./t1.*a + p1;

p(:,(temp>1|isinf(temp)|temp<=0)) = nan;

end

