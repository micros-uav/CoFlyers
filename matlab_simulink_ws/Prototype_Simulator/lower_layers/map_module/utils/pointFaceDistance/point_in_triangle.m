function in = point_in_triangle(p,a,b,c)
%POINT_IN_TRIANGLE Summary of this function goes here
%   Detailed explanation goes here

in = same_side(p,a, b,c) & same_side(p,b, a,c) & same_side(p,c, a,b);

end

