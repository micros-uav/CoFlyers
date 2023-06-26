function model_struct = update_map3d_user(t, dt, model_struct_in, id)
%UPDATE_MAP3D_USER Summary of this function goes here
%   Detailed explanation goes here
model_struct = model_struct_in;

persistent velocity

if id ==1
    position = model_struct(1:3);
    rotation = model_struct(4:6);
    scale = model_struct(7:9);
    color = model_struct(10:12);
    alpha = model_struct(13);

    %==============Change===========%
    if isempty(velocity)||t==0
        velocity = [0.1;0;0];
    end
    if position(1) > 1.5
        velocity =  -[0.1;0;0];
    elseif position(1) < -1.5
        velocity =  [0.1;0;0];
    end
    position = position + velocity*dt;
    rotation(1) = rotation(1) + 5*dt;
    %==============================%

    model_struct = [position;rotation;scale;color;alpha];
end

end

