function [states,dstates] = initialize_states(position0, motion_model_type)
%FLOCKING_MODEL_INITIALIZE_STATES Initialize states
%

coder.varsize('states');
coder.varsize('dstates');

number = size(position0,2);

switch motion_model_type
    case 'quadcopter' %quadcopter model
        velocity = [(rand(2,number)-0.5)*0.001;zeros(1,number)];
        qwxyz = zeros(4,number); qwxyz(1,:) = 1;
        pqr = zeros(3,number);
        motor_speed = zeros(4,number);
        states = [position0;velocity;qwxyz;pqr;motor_speed];
        dstates = states*0;
    otherwise %pass-mass and pass-mass-rotation
        velocity = (rand(3,number)-0.5)*0.001;
        states = [position0;velocity];
        dstates = states*0;
end



end

