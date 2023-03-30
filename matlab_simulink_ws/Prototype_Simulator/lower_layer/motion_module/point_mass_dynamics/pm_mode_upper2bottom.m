function mode_bottom = pm_mode_upper2bottom(mode_upper)
%QM_MODE_UPPER2BOTTOM Summary of this function goes here
%   Detailed explanation goes here

%   control_mode judges the control mode by binary bit 
%    0b000001: add position control, horizontal
%    0b000010: add velocity feedforword, horizontal
%    0b000100: add acceleration feedforword, horizontal
%    0b001000: add position control, vertical
%    0b010000: add velocity feedforword, vertical
%    0b100000: add acceleration feedforword, vertical

TAKEOFF_TYPE = 2;
HOVER_TYPE = 3;
LAND_TYPE = 4;
POSITION_CONTROL_TYPE = 5;
VELOCITY_CONTROL_TYPE = 6;
VELOCITY_HORIZONTAL_CONTROL_TYPE = 7;
ACCELERATION_CONTROL_TYPE = 8;

POSITION_CONTROL_TYPE_b = 0b001001;
VELOCITY_CONTROL_TYPE_b = 0b010010;
VELOCITY_HORIZONTAL_CONTROL_TYPE_b = 0b001010;
ACCELERATION_CONTROL_TYPE_b = 0b100100;


all_type_u = [TAKEOFF_TYPE;
HOVER_TYPE;
LAND_TYPE;
POSITION_CONTROL_TYPE;
VELOCITY_CONTROL_TYPE;
VELOCITY_HORIZONTAL_CONTROL_TYPE;
ACCELERATION_CONTROL_TYPE];

all_type_b = [POSITION_CONTROL_TYPE_b;   
POSITION_CONTROL_TYPE_b;
POSITION_CONTROL_TYPE_b;
POSITION_CONTROL_TYPE_b;
VELOCITY_CONTROL_TYPE_b
VELOCITY_HORIZONTAL_CONTROL_TYPE_b
ACCELERATION_CONTROL_TYPE_b];

[~,ind] = ismember(mode_upper,all_type_u);
mode_bottom = all_type_b(ind);
end

