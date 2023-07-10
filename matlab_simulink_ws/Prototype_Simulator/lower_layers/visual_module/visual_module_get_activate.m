function in = visual_module_get_activate(sample_time_motion, t, start_and_end)
%VISUAL_MODULE_GET_ACTIVATE Summary of this function goes here
%   Detailed explanation goes here

file_name_param = 'visual_module_parameters';
[~,str_core] = get_multi_core_value();
fun_params = str2func([file_name_param, str_core]);

[activate_plot,...
time_interval_plot] = fun_params();

rate_draw     = ceil(time_interval_plot/sample_time_motion);
count_now = round(t/sample_time_motion+1);

if (mod(count_now,rate_draw) == 0 || start_and_end) && activate_plot 
    in = true;
else
    in = false;
end

end