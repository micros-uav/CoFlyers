function [result1,result2] = parseXML2(xml_struct, result1_name)
        %PARSEXML2 Summary of this function goes here
        %   Detailed explanation goes here

        [result1,result2] = get_children_recurion(xml_struct, result1_name);

        function [result1, result2] = get_children_recurion(xml_struct, field_name_pre)
            result2 = string([]);
            if isempty(xml_struct)
                result1 = [];
            else
                result1 = struct();
            end
            count_model = 1;
            for i = 1:length(xml_struct.Children)
                % Deal with models
                name_str = xml_struct.Children(i).Name;
                if strcmp(name_str,"model") && strcmp(field_name_pre,strcat(result1_name,".map"))
                    name_str = strcat(name_str,num2str(count_model));
                    count_model = count_model + 1;
                end
                % Get the full name of the current field.
                if isempty(field_name_pre)
                    fullfield_str = string(name_str);
                else
                    fullfield_str = strcat(field_name_pre,'.',name_str);
                end
                % Get Value
                if ~isempty(xml_struct.Children(i).Attributes)
                    str_value = string(xml_struct.Children(i).Attributes.Value);
                else
                    str_value = [];
                end
                if ~isempty(str_value)
                    try
                        eval(strcat("result1.(name_str) = [",str_value,"];"));
                    catch
                        % Need reference
                        %  Declare variable names in the structure in advance 
                        temp_2 = strsplit(name_str,"-");
                        for j = 1:length(temp_2)
                            result1.(temp_2{j}) = str_value;
                        end
                        % 
                        result2 = [result2, [str_value;fullfield_str]];
                    end
                end

                %%% Get submodule
                if ~isempty(xml_struct.Children(i).Children)
                    [a,b] = get_children_recurion(xml_struct.Children(i),fullfield_str);
                    result1.(name_str) = a;
                    result2 = [result2,b];
                end
            end
        end
    end