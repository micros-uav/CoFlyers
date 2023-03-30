function array_all = get_array_slice(array_sub, num1, num2)
%GET_ARRAY_SLICE Summary of this function goes here
%   Detailed explanation goes here
num3 = length(array_sub);
array_all = repmat(array_sub,1,num1) + reshape(repmat((0:num1-1)*num2,num3,1),1,num3*num1);

end

