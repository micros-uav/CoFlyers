function [states,dstates] = initialize_states(number,...
    xrange,yrange,flag_motion_model,flag_alg,dim_alg)
%FLOCKING_MODEL_INITIALIZE_STATES Initialize states
%   此处显示详细说明

r_rep0 = 1.0;

delta_r = r_rep0;        

NSqrt = ceil(sqrt(number));
if dim_alg == 2
% Distribution centered on the origin
% position = [(mod(0:number-1,NSqrt) - mod(number-1,NSqrt)/2)*delta_r;
%     (floor((0:number-1)/NSqrt) - floor((number-1)/NSqrt)/2)*delta_r;
%     zeros(1,number)]; 
% y center
position = [(mod(0:number-1,NSqrt))*delta_r + xrange(1) + delta_r;
    (floor((0:number-1)/NSqrt) - floor((number-1)/NSqrt)/2)*delta_r;
    zeros(1,number)]; 
% x center
% position = [(mod(0:number-1,NSqrt) - mod(number-1,NSqrt)/2)*delta_r;
%     (floor((0:number-1)/NSqrt))*delta_r + yrange(1) + delta_r;
%     zeros(1,number)]; 
% corner
% position = [(mod(0:number-1,NSqrt))*delta_r + xrange(1) + delta_r;
%     (floor((0:number-1)/NSqrt))*delta_r + yrange(1) + delta_r;
%     zeros(1,number)]; 
else
    position = rand(3,number) * number^(1/3)*3.0;

end

switch flag_motion_model
    case 1 %quadcopter model

        velocity = [rand(2,number)*0.001;zeros(1,number)];
        qwxyz = zeros(4,number); qwxyz(1,:) = 1;
        pqr = zeros(3,number);
         motor_speed = zeros(4,number);
        states = [position;velocity;qwxyz;pqr;motor_speed];
        dstates = states*0;
    otherwise %pass-mass and pass-mass-rotation
        velocity = [rand(2,number)*0.001;zeros(1,number)];
        states = [position;velocity];
        dstates = states*0;
end

end

