function performances_mean = model_swarm_repeat(mode_simulation, ...
    parameters_op, parameters_bp, Nr, txtName)
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

parameters_setting = parameters_setting_get([],mode_simulation);

%%% Run the model once to get the dimension of the output
values = model_swarm(parameters_setting, parameters_op, parameters_bp,1);
values_all = zeros([length(values),Nr]);
values_all(:,1) = values;

%%% Repeat simulation
for i = 2:Nr
    values = model_swarm(parameters_setting, parameters_op, parameters_bp,i);
    values_all(:,i) = values;
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
        for i = 1:length(parameters_op)
            fprintf(myTxT,"param_%d\t",i);
        end
        for i = 1:length(value_mean)
            fprintf(myTxT,"value_%d\t",i);
        end
        fprintf(myTxT,"\n");
    end
    for i = 1:length(parameters_op)
        fprintf(myTxT,"%f\t",parameters_op(i));
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