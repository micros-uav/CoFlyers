function [FSpeed,FColl,FWall,FCorr,F] = fitness_function_combine(vFlock,aTol,vTol,rTol,phiVel, ...
    phiColl,phiWall,phiCorr)
% vTol = 1.5/4*vFlock;  aTol = 0.00003;   rTol = 2;
% FSpeed = fitness_function_1(phiVel,vFlock,vTol);
% FColl = fitness_function_3(phiColl,aTol);
FSpeed = heaviside(phiVel)*phiVel;
% FColl = (1 - phiColl)^50;
FColl = fitness_function_3(phiColl,aTol);
% FWall = fitness_function_2(phiWall,rTol);
% FWall = fitness_function_3(phiWall,aTol);
FWall = fitness_function_3(phiWall,aTol);
% FWall = (1 - phiWall)^50;
FCorr = heaviside(phiCorr)*phiCorr;
F = 1 - FSpeed*FColl*FWall*FCorr;
end
