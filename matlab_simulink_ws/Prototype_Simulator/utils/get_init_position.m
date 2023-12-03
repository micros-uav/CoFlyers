function position0 = get_init_position(delta,number)
%GET_INIT_POSITION 
%   
n_sqrt = floor(sqrt(number));
position0 = [mod(0:(number-1), n_sqrt)*delta - n_sqrt/2*delta + 0.5;
    floor((0:(number-1))./n_sqrt)*delta - n_sqrt/2*delta + 0.5;
    zeros(1,number)];

% position_real = [0.5, 0.5, -0.5, -0.5;
%     0.5,-0.5,-0.5,0.5];
id_real = [37, 29, 28, 36];

for i = 1:length(id_real)
    temp = position0(:,id_real(i));
    position0(:,id_real(i)) = position0(:,i);
    position0(:,i) = temp;
end

end

