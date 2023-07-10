function [flag_parallel, str_core] = get_multi_core_value()
%GET_MULTI_CORE_VALUE Summary of this function goes here
%  Get the id of the processor core running this function.
%  When performing multi-core computing, the creation and calling of parameter functions in each core should be splited.



if coder.target('MATLAB')
    temp = getCurrentTask();
    id_core = [];
    flag_parallel = false;
    if ~isempty(temp)
        id_core = temp.ID;
        flag_parallel = true;
    end

    if flag_parallel
        str_core = ['_',num2str(id_core)];
    else
        str_core = '';
    end
else
    flag_parallel = false;
    str_core = '';
end


end