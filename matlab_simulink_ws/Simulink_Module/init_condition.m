sample_time_control = 1/30;
number = 10;
flag_alg = 0;
parameters_map = map_module_parameters(number,flag_alg,[]);
[map_lines,map_grid] = map_module_generate(parameters_map);
parameters_flocking = swarm_module_parameters(number,map_lines,flag_alg,[],[]);
parameters_sensor_env = sensor_env_module_parameters(0,[]);