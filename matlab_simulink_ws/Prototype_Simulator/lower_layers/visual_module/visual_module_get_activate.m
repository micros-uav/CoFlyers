function in = visual_module_get_activate(sample_time_motion, t, start_and_end)
%VISUAL_MODULE_GET_ACTIVATE Summary of this function goes here
%   Detailed explanation goes here

[activate_plot,...
time_interval_plot] = visual_module_parameters();

rate_draw     = ceil(time_interval_plot/sample_time_motion);
count_now = round(t/sample_time_motion+1);

if (mod(count_now,rate_draw) == 0 || start_and_end) && activate_plot 
    in = true;
else
    in = false;
end

end