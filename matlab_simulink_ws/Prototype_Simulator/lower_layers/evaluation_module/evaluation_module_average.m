function values = evaluation_module_average(evaluation_metric_type, time_series, values_series)
%EVALUATION_MODEL_AVERAGE 
% Automatically generated by read_parameter_xml.m
% Every time read_parameter_xml.m is run, this function will be generated
switch evaluation_metric_type
	case 'evaluation_0'
		values = evaluation_0_module_average(time_series, values_series);
	otherwise
		values = [];
end

end