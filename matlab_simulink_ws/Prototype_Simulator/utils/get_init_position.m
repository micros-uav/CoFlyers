function position0 = get_init_position(delta, number, x0, y0, z0)
%GET_INIT_POSITION 
%   
n_sqrt = floor(sqrt(number));

position0 = [mod(0:(number-1), n_sqrt)*delta - n_sqrt/2*delta + 0.5;
    floor((0:(number-1))./n_sqrt)*delta - n_sqrt/2*delta + 0.5;
    zeros(1,number)+z0];

x_m = mean(position0(1,:));
y_m = mean(position0(2,:));

position0(1,:) = position0(1,:)-x_m+x0;
position0(2,:) = position0(2,:)-y_m+y0;
end

