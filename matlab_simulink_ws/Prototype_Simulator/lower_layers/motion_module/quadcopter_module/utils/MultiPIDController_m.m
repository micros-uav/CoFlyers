function [output,error,error_diff,error_integral] = MultiPIDController_m(error,reset,sampleTime,kp,kd,ki,c,lb,ub,ep,ed,ei)
% persistent ep ed ei
% if isempty(ep)
%     ep = error;
% end
% if isempty(ei)
%     ei = error*0;
% end
% if isempty(ed)
%     ed = error*0;
% end

if reset == 1
    ei = error*0;
end

error_diff = (error - ep)/sampleTime;
error_diff = ed * (1 - c) + error_diff;

error_integral = ei + (error + ep)/2*sampleTime;

output = kp.*error + kd.*error_diff + ki.*error_integral;

b_array = repmat(lb,1,size(error,2));
temp = output < b_array;
output(temp) = b_array(temp);

b_array = repmat(ub,1,size(error,2));
temp = output > b_array;
output(temp) = b_array(temp);

% ep = error;
% ed = error_diff;
% ei = error_integral;
end