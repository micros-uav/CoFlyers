function write_xml_from_paras_map(x_range,x_color,x_alpha,activate_x,...
    y_range,y_color,y_alpha,activate_y,...
    z_range,z_color,z_alpha,activate_z,...
    map3d_struct, model_stls)
%WRITE_XML_FROM_PARAS_MAP Write the map parameters to a XML file in the
%CoFlyers format.
%===================%
ind_range = find(map3d_struct(15,:)==-2);
map3d_struct(:,ind_range) = [];
model_stls(ind_range) = [];
%===================%

time_now_string = char(datetime('now','TimeZone','local','Format','y_M_d_H_m_s_SSS'));

file_fullpath = strcat("parameters_map_",time_now_string,".xml");
file_id = fopen(file_fullpath,'w');

fprintf(file_id,"<CoFlyers>\n");
fprintf(file_id,"\t<map>\n");
fprintf(file_id,"\t\t<activate value=""1\""/>\n");
fprintf(file_id,"\t\t<activate_periodic_boundary value=""0\""/>\n");
if activate_x
    fprintf(file_id,"\t\t<x_range value=""%.3f;%.3f\""/>\n",x_range(1),x_range(2));
else
    fprintf(file_id,"\t\t<x_range value=""\""/>\n");
end
if activate_y
    fprintf(file_id,"\t\t<y_range value=""%.3f;%.3f\""/>\n",y_range(1),y_range(2));
else
    fprintf(file_id,"\t\t<y_range value=""\""/>\n");
end
if activate_z
    fprintf(file_id,"\t\t<z_range value=""%.3f;%.3f\""/>\n",z_range(1),z_range(2));
else
    fprintf(file_id,"\t\t<z_range value=""\""/>\n");
end

fprintf(file_id,"\t\t<x_color value=""%.3f;%.3f;%.3f\""/>\n",x_color(1),x_color(2),x_color(3));
fprintf(file_id,"\t\t<y_color value=""%.3f;%.3f;%.3f\""/>\n",y_color(1),y_color(2),y_color(3));
fprintf(file_id,"\t\t<z_color value=""%.3f;%.3f;%.3f\""/>\n",z_color(1),z_color(2),z_color(3));
fprintf(file_id,"\t\t<x_alpha value=""%.3f\""/>\n",x_alpha);
fprintf(file_id,"\t\t<y_alpha value=""%.3f\""/>\n",y_alpha);
fprintf(file_id,"\t\t<z_alpha value=""%.3f\""/>\n",z_alpha);

fun_f = @(a)"["+strjoin(string([repmat('[',size(a,2),1),num2str(a',"%.3f;"),repmat(']',size(a,2),1)])',',')+"]";
fun_d = @(a)"["+strjoin(string([repmat('[',size(a,2),1),num2str(a',"%.0f;"),repmat(']',size(a,2),1)])',',')+"]";

model_stls_unique = unique(model_stls);
for i = 1:length(model_stls_unique)
    model_stl = model_stls_unique(i);
    ind_same = find(strcmp(model_stls,model_stl));
    fprintf(file_id,"\t\t<model>\n");
    fprintf(file_id,"\t\t\t<stl value=""%s""/>\n",model_stl);
    fprintf(file_id,"\t\t\t<position value=""%s""/> \n",fun_f(map3d_struct(1:3,ind_same)));
    fprintf(file_id,"\t\t\t<rotation value=""%s""/> \n",fun_f(map3d_struct(4:6,ind_same)));
    fprintf(file_id,"\t\t\t<scale value=""%s""/> \n",fun_f(map3d_struct(7:9,ind_same)));
    fprintf(file_id,"\t\t\t<color value=""%s""/> \n",fun_f(map3d_struct(10:12,ind_same)));
    fprintf(file_id,"\t\t\t<alpha value=""%s""/> \n",fun_f(map3d_struct(13,ind_same)));
    fprintf(file_id,"\t\t\t<static value=""%s""/> \n",fun_d(map3d_struct(14,ind_same)));
    fprintf(file_id,"\t\t\t<id value=""%s""/> \n",fun_d(map3d_struct(15,ind_same)));
    fprintf(file_id,"\t\t</model>\n");

end


% for i = 1:size(map3d_struct,2)
%     % [position;rotation;scale;color;alpha;static;id;ind_start;ind_end]
%     fprintf(file_id,"\t\t<model>\n");
%     fprintf(file_id,"\t\t\t<stl value=""%s""/>\n",model_stls(i));
%     fprintf(file_id,"\t\t\t<position value=""[[5;5;0]]""/> \n");
%     fprintf(file_id,"\t\t\t<rotation value=""0;0;0""/>\n");
%     fprintf(file_id,"\t\t\t<scale value=""1;1;1""/>\n");
%     fprintf(file_id,"\t\t\t<color value=""0.5;0.5;0.5""/>\n");
%     fprintf(file_id,"\t\t\t<alpha value=""1""/>\n");
%     fprintf(file_id,"\t\t\t<static value=""0""/> \n");
%     fprintf(file_id,"\t\t\t<id value=""0""/>\n");
%     fprintf(file_id,"\t\t<model/>\n");
% end


fprintf(file_id,"\t</map>\n");
fprintf(file_id,"</CoFlyers>");
fclose(file_id);
end

