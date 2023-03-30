function dir_name = get_dir_name_from_mode(mode_simulation)
%GET_DIR_NAME_FROM_MODE 此处显示有关此函数的摘要
%   此处显示详细说明

switch mode_simulation
    case 0
        dir_name = 'data_save/rapid_prototyping';
    case 1
        dir_name = 'data_save/auto_tuning';
    case 2
        dir_name = 'data_save/batch_processing';
    otherwise
        dir_name = [];
end

end

