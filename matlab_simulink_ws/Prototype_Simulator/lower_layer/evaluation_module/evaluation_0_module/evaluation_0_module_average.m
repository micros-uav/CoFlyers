function values = evaluation_0_module_average(values_series,parameters_evalue)
%EVALUATION_MODEL_ONE 

[v_flock,~,vTol,aTol,rTol] =...
    evaluation_0_module_parameters_deal(parameters_evalue);

phiCorr = mean(values_series(1,:));
phiVel  = mean(values_series(2,:));
phiColl = mean(values_series(3,:));
phiWall = mean(values_series(4,:));
phiMND = mean(values_series(5,:));

[~,~,~,~,F] = fitness_function_combine(v_flock,aTol,vTol,rTol,phiVel, ...
    phiColl,phiWall,phiCorr);

values = [F;phiCorr;phiVel;phiColl;phiWall;phiMND];
end