function performances_mean = model_swarm_repeat(mode_simulation, ...
    parameters_op, parameters_bp, Nr, txtName, flag_parallel)
%MODEL_SWARM_REPEAT repeats running "model_swarm" to get the average of Nr
%simulations
%INPUT:
%   Nr: repeated times.
%   txtName: the name of txt file to save performances when auto-tuning.
%   The description of other parameters is the same as "model_swarm".

%%% Exception handling
if Nr < 1
    performances_mean = [];
    return
end

%%% Repeat simulation
values_all = [];
if flag_parallel
    parfor i = 1:Nr
        values = model_swarm([], parameters_op, parameters_bp,i,mode_simulation);
        values_all(:,i) = values;
    end
else
    for i = 1:Nr
        values = model_swarm([], parameters_op, parameters_bp,i,mode_simulation);
        values_all(:,i) = values;
    end
end
value_mean = mean(values_all,2);

%%% Save the ouput performances and parameters when auto-tuning
if ~isempty(txtName) && mode_simulation == 1
    %  Get the id of the processor core running this function
    t = getCurrentTask();
    id = 1;
    if ~isempty(t)
        id = t.ID;
    end
    % Write data
    whole_name = [txtName,'_w',num2str(id),'.txt'];
    first_create = isempty(dir(whole_name));
    myTxT = fopen(whole_name,'a');
    if first_create
        for i = 1:length(parameters_op.param_name_s)
            fprintf(myTxT,"%s\t",parameters_op.param_name_s{i});
        end
        for i = 1:length(value_mean)
            fprintf(myTxT,"value_%d\t",i);
        end
        fprintf(myTxT,"\n");
    end
    for i = 1:length(parameters_op.param_name_s)
%         fprintf(myTxT,"%f\t",parameters_op.param_value_s{i});
        fprintf(myTxT,"%s\t",parameters_op.param_value_s{i});
    end
    for i = 1:length(value_mean)
        fprintf(myTxT,"%f\t",value_mean(i));
    end
    fprintf(myTxT,"\n");
    fclose(myTxT);
end
if mode_simulation == 1
    disp(value_mean');
    performances_mean = value_mean(1);
else
    performances_mean = value_mean;
end
end