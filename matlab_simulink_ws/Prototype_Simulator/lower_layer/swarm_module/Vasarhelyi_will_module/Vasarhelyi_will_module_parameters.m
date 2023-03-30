function parameters_flocking = Vasarhelyi_will_module_parameters(number_h,map_lines)
%VASARHELYI_MODULE_PARAMETERS Summary of this function goes here
%   Detailed explanation goes here

%%% Parameters to be optimized							
rRep0   = 0.861723;										
pRep    = 0.923553;	
rFrict0 = 5.693489;	
CFrict  = 0.155726;	
vFrict  = 0.057981;	
pFrict  = 4.506540;	
aFrict  = 0.401540;	
rShill0 = 0.968532;	
vShill  = 0.843399;	
pShill  = 1.980570;	
aShill  = 0.223193;
tau_will = 1.0;
% r_wall = 2;
%%% Other parameters
% flocking speed
v_flock = 0.2;
% 
rCom = 4;

temp = magic(ceil(sqrt(number_h)));
heights = temp(1:number_h)/max(temp,[],'all')*0.2+0.5;

v_max = v_flock*1.2;

% % dr = rCom/10;
dr = 1;
[numS,posShill,velShill] = generateShills(map_lines,dr);

%%% Merge all parameters to an array
parameters_flocking = [
    rCom;
    v_flock;
    tau_will
    rRep0;
    pRep;
    rFrict0;
    CFrict;
    vFrict;
    pFrict;
    aFrict;
    rShill0;	
    vShill;
    pShill;
    aShill;
    v_max;
    number_h;
    heights(:);
    numS;
    posShill(:);
    velShill(:)];

%Automatically allocate equidistant(dr) virtual particles according to the
% start and end points of map
    function [numS,posS,velS] = generateShills(map_lines,dr)
        posS = [];
        velS = [];
        numS = 0;
        for m = 1:size(map_lines,2)
            ms = map_lines(1:2,m);
            me = map_lines(3:4,m);
            dis = norm(me - ms);
            numS_one = ceil(dis/dr)+1;
            posS_m = linspacePoints(ms,me,numS_one);
            posS = [posS,posS_m];
            vv = -map_lines(5:6,m);
            velS = [velS,repmat(vv,1,numS_one)];
            numS = numS + numS_one;
        end
    end
    function points = linspacePoints(point1,point2,num)
        points = zeros(length(point1),num);
        for ppp = 1:length(point1)
            points(ppp,:) = linspace(point1(ppp),point2(ppp),num);
        end
    end

end

