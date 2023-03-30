clc;close all;clear;
%% Settings of auto-tuning 
% Lower boundary and upper boundary of optimization
flag_use_parallel     = false;
repeat_times        = 10; %Times of experimental repetitions used to average multiple simulations
%====[rRep0====pRep=rFrict0====CFrict====vFrict====pFrict====aFrict====rShill0===vShill====pShill====aShill]
lower_boudaries = [0.2000   0.01   00.1000    0.0100    0.0100    0.1000    0.0100    0.0100   0.0100    0.0100    0.0100];
upper_boudaries = [2.0000    1.0   10.0000     0.500    0.2000    10.000    1.000     1.000    2.000    10.000     1.000];
order_s = [3:13];
flag_alg_opt = 1;
%% Running auto-tuning
x_opt = run_auto_tuning(flag_use_parallel,repeat_times,...
    lower_boudaries, upper_boudaries, order_s, flag_alg_opt);