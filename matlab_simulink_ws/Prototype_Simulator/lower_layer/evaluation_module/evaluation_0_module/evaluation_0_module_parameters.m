function parameters_evalue = evaluation_0_module_parameters(v_flock)
%EVALUATION_MODEL_PARAMETERS 

vTol = 1.5/4*v_flock;  aTol = 0.00003;   rTol = 2;

r_coll_evalue = 0.4;

parameters_evalue = [v_flock;r_coll_evalue;vTol;aTol;rTol];

end

