function f = fitness_function_1(phi,phi0,d)
s = 0;
if phi < phi0 - d
    s = 1;
elseif phi < phi0
    s = 0.5 * (1 - cos(pi/d * (phi - phi0)));
end
f = 1-s;
end