function [v_flock,rColl,vTol,aTol,rTol] =...
    evaluation_0_module_parameters_deal(parameters_evalue)
%EVALUATION_MODEL_PARAMETERS

count = 1;
v_flock  = parameters_evalue(count);count = count + 1;
rColl  = parameters_evalue(count);count = count + 1;
vTol   = parameters_evalue(count);count = count + 1;
aTol   = parameters_evalue(count);count = count + 1;
rTol   = parameters_evalue(count);count = count + 1;
end

