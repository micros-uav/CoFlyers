function [activate_plot,...
time_interval_plot,...
activate_trajectory,...
follow_agent,...
activate_save_figure,...
activate_save_video,...
dim_visual,...
time_interval_trajectory,...
video_speed,...
x_range,...
y_range,...
z_range,...
legend_name,...
font_size,...
font_size_sub,...
marker_size,...
background_color,...
activate_BD_1,...
len_arm,...
cmap_terrain,...
cmap_traj,...
T_end] = visual_module_parameters()
%VISUAL_MODULE_PARAMETERS 
% Automatically generated by read_parameter_xml.m
% Every time read_parameter_xml.m is run, this function will be generated
activate_plot = 1.000000000000;
time_interval_plot = 1.000000000000;
activate_trajectory = 1.000000000000;
follow_agent = 0.000000000000;
activate_save_figure = 0.000000000000;
activate_save_video = 0.000000000000;
dim_visual = 3.000000000000;
time_interval_trajectory = 20.000000000000;
video_speed = 10.000000000000;
x_range = [-236.000000000000
280.000000000000];
y_range = [-185.000000000000
137.000000000000];
z_range = [0.000000000000
252.000000000000];
legend_name = ['$', '\', 'p', 'h', 'i', '^', '{', 'c', 'o', 'r', 'r', '}', '$', '|', '|', '|', '$', '\', 'p', 'h', 'i', '^', '{', 'v', 'e', 'l', '}', '$', '|', '|', '|', '$', '\', 'p', 'h', 'i', '^', '{', 'c', 'o', 'l', 'l', '}', '$', '|', '|', '|', '$', '\', 'p', 'h', 'i', '^', '{', 'w', 'a', 'l', 'l', '}', '$', '|', '|', '|', '$', '\', 'p', 'h', 'i', '^', '{', 'M', 'N', 'D', '}', '$'];
font_size = 14.000000000000;
font_size_sub = 10.000000000000;
marker_size = 10.000000000000;
background_color = 'w';
activate_BD_1 = 0.000000000000;
len_arm = 0.500000000000;
cmap_terrain = [0.900000000000, 0.944700000000, 0.974100000000
0.896500000000, 0.942700000000, 0.973200000000
0.892900000000, 0.940800000000, 0.972300000000
0.889400000000, 0.938800000000, 0.971400000000
0.885900000000, 0.936900000000, 0.970400000000
0.882400000000, 0.934900000000, 0.969500000000
0.878800000000, 0.933000000000, 0.968600000000
0.875300000000, 0.931000000000, 0.967700000000
0.871800000000, 0.929100000000, 0.966800000000
0.868200000000, 0.927100000000, 0.965900000000
0.864700000000, 0.925200000000, 0.965000000000
0.861200000000, 0.923200000000, 0.964000000000
0.857600000000, 0.921300000000, 0.963100000000
0.854100000000, 0.919300000000, 0.962200000000
0.850600000000, 0.917400000000, 0.961300000000
0.847100000000, 0.915400000000, 0.960400000000
0.843500000000, 0.913500000000, 0.959500000000
0.840000000000, 0.911500000000, 0.958600000000
0.836500000000, 0.909600000000, 0.957600000000
0.832900000000, 0.907600000000, 0.956700000000
0.829400000000, 0.905700000000, 0.955800000000
0.825900000000, 0.903700000000, 0.954900000000
0.822400000000, 0.901800000000, 0.954000000000
0.818800000000, 0.899800000000, 0.953100000000
0.815300000000, 0.897900000000, 0.952200000000
0.811800000000, 0.895900000000, 0.951200000000
0.808200000000, 0.894000000000, 0.950300000000
0.804700000000, 0.892000000000, 0.949400000000
0.801200000000, 0.890100000000, 0.948500000000
0.797600000000, 0.888100000000, 0.947600000000
0.794100000000, 0.886100000000, 0.946700000000
0.790600000000, 0.884200000000, 0.945800000000
0.787100000000, 0.882200000000, 0.944800000000
0.783500000000, 0.880300000000, 0.943900000000
0.780000000000, 0.878300000000, 0.943000000000
0.776500000000, 0.876400000000, 0.942100000000
0.772900000000, 0.874400000000, 0.941200000000
0.769400000000, 0.872500000000, 0.940300000000
0.765900000000, 0.870500000000, 0.939400000000
0.762400000000, 0.868600000000, 0.938400000000
0.758800000000, 0.866600000000, 0.937500000000
0.755300000000, 0.864700000000, 0.936600000000
0.751800000000, 0.862700000000, 0.935700000000
0.748200000000, 0.860800000000, 0.934800000000
0.744700000000, 0.858800000000, 0.933900000000
0.741200000000, 0.856900000000, 0.933000000000
0.737600000000, 0.854900000000, 0.932100000000
0.734100000000, 0.853000000000, 0.931100000000
0.730600000000, 0.851000000000, 0.930200000000
0.727100000000, 0.849100000000, 0.929300000000
0.723500000000, 0.847100000000, 0.928400000000
0.720000000000, 0.845200000000, 0.927500000000
0.716500000000, 0.843200000000, 0.926600000000
0.712900000000, 0.841300000000, 0.925700000000
0.709400000000, 0.839300000000, 0.924700000000
0.705900000000, 0.837400000000, 0.923800000000
0.702400000000, 0.835400000000, 0.922900000000
0.698800000000, 0.833400000000, 0.922000000000
0.695300000000, 0.831500000000, 0.921100000000
0.691800000000, 0.829500000000, 0.920200000000
0.688200000000, 0.827600000000, 0.919300000000
0.684700000000, 0.825600000000, 0.918300000000
0.681200000000, 0.823700000000, 0.917400000000
0.677600000000, 0.821700000000, 0.916500000000
0.674100000000, 0.819800000000, 0.915600000000
0.670600000000, 0.817800000000, 0.914700000000
0.667100000000, 0.815900000000, 0.913800000000
0.663500000000, 0.813900000000, 0.912900000000
0.660000000000, 0.812000000000, 0.911900000000
0.656500000000, 0.810000000000, 0.911000000000
0.652900000000, 0.808100000000, 0.910100000000
0.649400000000, 0.806100000000, 0.909200000000
0.645900000000, 0.804200000000, 0.908300000000
0.642400000000, 0.802200000000, 0.907400000000
0.638800000000, 0.800300000000, 0.906500000000
0.635300000000, 0.798300000000, 0.905500000000
0.631800000000, 0.796400000000, 0.904600000000
0.628200000000, 0.794400000000, 0.903700000000
0.624700000000, 0.792500000000, 0.902800000000
0.621200000000, 0.790500000000, 0.901900000000
0.617600000000, 0.788600000000, 0.901000000000
0.614100000000, 0.786600000000, 0.900100000000
0.610600000000, 0.784700000000, 0.899100000000
0.607100000000, 0.782700000000, 0.898200000000
0.603500000000, 0.780800000000, 0.897300000000
0.600000000000, 0.778800000000, 0.896400000000
0.596500000000, 0.776800000000, 0.895500000000
0.592900000000, 0.774900000000, 0.894600000000
0.589400000000, 0.772900000000, 0.893700000000
0.585900000000, 0.771000000000, 0.892700000000
0.582400000000, 0.769000000000, 0.891800000000
0.578800000000, 0.767100000000, 0.890900000000
0.575300000000, 0.765100000000, 0.890000000000
0.571800000000, 0.763200000000, 0.889100000000
0.568200000000, 0.761200000000, 0.888200000000
0.564700000000, 0.759300000000, 0.887300000000
0.561200000000, 0.757300000000, 0.886300000000
0.557600000000, 0.755400000000, 0.885400000000
0.554100000000, 0.753400000000, 0.884500000000
0.550600000000, 0.751500000000, 0.883600000000
0.547100000000, 0.749500000000, 0.882700000000
0.543500000000, 0.747600000000, 0.881800000000
0.540000000000, 0.745600000000, 0.880900000000
0.536500000000, 0.743700000000, 0.879900000000
0.532900000000, 0.741700000000, 0.879000000000
0.529400000000, 0.739800000000, 0.878100000000
0.525900000000, 0.737800000000, 0.877200000000
0.522400000000, 0.735900000000, 0.876300000000
0.518800000000, 0.733900000000, 0.875400000000
0.515300000000, 0.732000000000, 0.874500000000
0.511800000000, 0.730000000000, 0.873500000000
0.508200000000, 0.728100000000, 0.872600000000
0.504700000000, 0.726100000000, 0.871700000000
0.501200000000, 0.724200000000, 0.870800000000
0.497600000000, 0.722200000000, 0.869900000000
0.494100000000, 0.720200000000, 0.869000000000
0.490600000000, 0.718300000000, 0.868100000000
0.487100000000, 0.716300000000, 0.867100000000
0.483500000000, 0.714400000000, 0.866200000000
0.480000000000, 0.712400000000, 0.865300000000
0.476500000000, 0.710500000000, 0.864400000000
0.472900000000, 0.708500000000, 0.863500000000
0.469400000000, 0.706600000000, 0.862600000000
0.465900000000, 0.704600000000, 0.861700000000
0.462400000000, 0.702700000000, 0.860700000000
0.458800000000, 0.700700000000, 0.859800000000
0.455300000000, 0.698800000000, 0.858900000000
0.451800000000, 0.696800000000, 0.858000000000
0.448200000000, 0.694900000000, 0.857100000000
0.444700000000, 0.692900000000, 0.856200000000
0.441200000000, 0.691000000000, 0.855300000000
0.437600000000, 0.689000000000, 0.854400000000
0.434100000000, 0.687100000000, 0.853400000000
0.430600000000, 0.685100000000, 0.852500000000
0.427100000000, 0.683200000000, 0.851600000000
0.423500000000, 0.681200000000, 0.850700000000
0.420000000000, 0.679300000000, 0.849800000000
0.416500000000, 0.677300000000, 0.848900000000
0.412900000000, 0.675400000000, 0.848000000000
0.409400000000, 0.673400000000, 0.847000000000
0.405900000000, 0.671500000000, 0.846100000000
0.402400000000, 0.669500000000, 0.845200000000
0.398800000000, 0.667500000000, 0.844300000000
0.395300000000, 0.665600000000, 0.843400000000
0.391800000000, 0.663600000000, 0.842500000000
0.388200000000, 0.661700000000, 0.841600000000
0.384700000000, 0.659700000000, 0.840600000000
0.381200000000, 0.657800000000, 0.839700000000
0.377600000000, 0.655800000000, 0.838800000000
0.374100000000, 0.653900000000, 0.837900000000
0.370600000000, 0.651900000000, 0.837000000000
0.367100000000, 0.650000000000, 0.836100000000
0.363500000000, 0.648000000000, 0.835200000000
0.360000000000, 0.646100000000, 0.834200000000
0.356500000000, 0.644100000000, 0.833300000000
0.352900000000, 0.642200000000, 0.832400000000
0.349400000000, 0.640200000000, 0.831500000000
0.345900000000, 0.638300000000, 0.830600000000
0.342400000000, 0.636300000000, 0.829700000000
0.338800000000, 0.634400000000, 0.828800000000
0.335300000000, 0.632400000000, 0.827800000000
0.331800000000, 0.630500000000, 0.826900000000
0.328200000000, 0.628500000000, 0.826000000000
0.324700000000, 0.626600000000, 0.825100000000
0.321200000000, 0.624600000000, 0.824200000000
0.317600000000, 0.622700000000, 0.823300000000
0.314100000000, 0.620700000000, 0.822400000000
0.310600000000, 0.618800000000, 0.821400000000
0.307100000000, 0.616800000000, 0.820500000000
0.303500000000, 0.614900000000, 0.819600000000
0.300000000000, 0.612900000000, 0.818700000000
0.296500000000, 0.610900000000, 0.817800000000
0.292900000000, 0.609000000000, 0.816900000000
0.289400000000, 0.607000000000, 0.816000000000
0.285900000000, 0.605100000000, 0.815000000000
0.282400000000, 0.603100000000, 0.814100000000
0.278800000000, 0.601200000000, 0.813200000000
0.275300000000, 0.599200000000, 0.812300000000
0.271800000000, 0.597300000000, 0.811400000000
0.268200000000, 0.595300000000, 0.810500000000
0.264700000000, 0.593400000000, 0.809600000000
0.261200000000, 0.591400000000, 0.808600000000
0.257600000000, 0.589500000000, 0.807700000000
0.254100000000, 0.587500000000, 0.806800000000
0.250600000000, 0.585600000000, 0.805900000000
0.247100000000, 0.583600000000, 0.805000000000
0.243500000000, 0.581700000000, 0.804100000000
0.240000000000, 0.579700000000, 0.803200000000
0.236500000000, 0.577800000000, 0.802200000000
0.232900000000, 0.575800000000, 0.801300000000
0.229400000000, 0.573900000000, 0.800400000000
0.225900000000, 0.571900000000, 0.799500000000
0.222400000000, 0.570000000000, 0.798600000000
0.218800000000, 0.568000000000, 0.797700000000
0.215300000000, 0.566100000000, 0.796800000000
0.211800000000, 0.564100000000, 0.795800000000
0.208200000000, 0.562200000000, 0.794900000000
0.204700000000, 0.560200000000, 0.794000000000
0.201200000000, 0.558300000000, 0.793100000000
0.197600000000, 0.556300000000, 0.792200000000
0.194100000000, 0.554300000000, 0.791300000000
0.190600000000, 0.552400000000, 0.790400000000
0.187100000000, 0.550400000000, 0.789400000000
0.183500000000, 0.548500000000, 0.788500000000
0.180000000000, 0.546500000000, 0.787600000000
0.176500000000, 0.544600000000, 0.786700000000
0.172900000000, 0.542600000000, 0.785800000000
0.169400000000, 0.540700000000, 0.784900000000
0.165900000000, 0.538700000000, 0.784000000000
0.162400000000, 0.536800000000, 0.783000000000
0.158800000000, 0.534800000000, 0.782100000000
0.155300000000, 0.532900000000, 0.781200000000
0.151800000000, 0.530900000000, 0.780300000000
0.148200000000, 0.529000000000, 0.779400000000
0.144700000000, 0.527000000000, 0.778500000000
0.141200000000, 0.525100000000, 0.777600000000
0.137600000000, 0.523100000000, 0.776700000000
0.134100000000, 0.521200000000, 0.775700000000
0.130600000000, 0.519200000000, 0.774800000000
0.127100000000, 0.517300000000, 0.773900000000
0.123500000000, 0.515300000000, 0.773000000000
0.120000000000, 0.513400000000, 0.772100000000
0.116500000000, 0.511400000000, 0.771200000000
0.112900000000, 0.509500000000, 0.770300000000
0.109400000000, 0.507500000000, 0.769300000000
0.105900000000, 0.505600000000, 0.768400000000
0.102400000000, 0.503600000000, 0.767500000000
0.098800000000, 0.501600000000, 0.766600000000
0.095300000000, 0.499700000000, 0.765700000000
0.091800000000, 0.497700000000, 0.764800000000
0.088200000000, 0.495800000000, 0.763900000000
0.084700000000, 0.493800000000, 0.762900000000
0.081200000000, 0.491900000000, 0.762000000000
0.077600000000, 0.489900000000, 0.761100000000
0.074100000000, 0.488000000000, 0.760200000000
0.070600000000, 0.486000000000, 0.759300000000
0.067100000000, 0.484100000000, 0.758400000000
0.063500000000, 0.482100000000, 0.757500000000
0.060000000000, 0.480200000000, 0.756500000000
0.056500000000, 0.478200000000, 0.755600000000
0.052900000000, 0.476300000000, 0.754700000000
0.049400000000, 0.474300000000, 0.753800000000
0.045900000000, 0.472400000000, 0.752900000000
0.042400000000, 0.470400000000, 0.752000000000
0.038800000000, 0.468500000000, 0.751100000000
0.035300000000, 0.466500000000, 0.750100000000
0.031800000000, 0.464600000000, 0.749200000000
0.028200000000, 0.462600000000, 0.748300000000
0.024700000000, 0.460700000000, 0.747400000000
0.021200000000, 0.458700000000, 0.746500000000
0.017600000000, 0.456800000000, 0.745600000000
0.014100000000, 0.454800000000, 0.744700000000
0.010600000000, 0.452900000000, 0.743700000000
0.007100000000, 0.450900000000, 0.742800000000
0.003500000000, 0.449000000000, 0.741900000000
0.000000000000, 0.447000000000, 0.741000000000];
cmap_traj = [1.000000000000, 0.000000000000, 0.000000000000
1.000000000000, 0.023437500000, 0.000000000000
1.000000000000, 0.046875000000, 0.000000000000
1.000000000000, 0.070312500000, 0.000000000000
1.000000000000, 0.093750000000, 0.000000000000
1.000000000000, 0.117187500000, 0.000000000000
1.000000000000, 0.140625000000, 0.000000000000
1.000000000000, 0.164062500000, 0.000000000000
1.000000000000, 0.187500000000, 0.000000000000
1.000000000000, 0.210937500000, 0.000000000000
1.000000000000, 0.234375000000, 0.000000000000
1.000000000000, 0.257812500000, 0.000000000000
1.000000000000, 0.281250000000, 0.000000000000
1.000000000000, 0.304687500000, 0.000000000000
1.000000000000, 0.328125000000, 0.000000000000
1.000000000000, 0.351562500000, 0.000000000000
1.000000000000, 0.375000000000, 0.000000000000
1.000000000000, 0.398437500000, 0.000000000000
1.000000000000, 0.421875000000, 0.000000000000
1.000000000000, 0.445312500000, 0.000000000000
1.000000000000, 0.468750000000, 0.000000000000
1.000000000000, 0.492187500000, 0.000000000000
1.000000000000, 0.515625000000, 0.000000000000
1.000000000000, 0.539062500000, 0.000000000000
1.000000000000, 0.562500000000, 0.000000000000
1.000000000000, 0.585937500000, 0.000000000000
1.000000000000, 0.609375000000, 0.000000000000
1.000000000000, 0.632812500000, 0.000000000000
1.000000000000, 0.656250000000, 0.000000000000
1.000000000000, 0.679687500000, 0.000000000000
1.000000000000, 0.703125000000, 0.000000000000
1.000000000000, 0.726562500000, 0.000000000000
1.000000000000, 0.750000000000, 0.000000000000
1.000000000000, 0.773437500000, 0.000000000000
1.000000000000, 0.796875000000, 0.000000000000
1.000000000000, 0.820312500000, 0.000000000000
1.000000000000, 0.843750000000, 0.000000000000
1.000000000000, 0.867187500000, 0.000000000000
1.000000000000, 0.890625000000, 0.000000000000
1.000000000000, 0.914062500000, 0.000000000000
1.000000000000, 0.937500000000, 0.000000000000
1.000000000000, 0.960937500000, 0.000000000000
1.000000000000, 0.984375000000, 0.000000000000
0.992187500000, 1.000000000000, 0.000000000000
0.968750000000, 1.000000000000, 0.000000000000
0.945312500000, 1.000000000000, 0.000000000000
0.921875000000, 1.000000000000, 0.000000000000
0.898437500000, 1.000000000000, 0.000000000000
0.875000000000, 1.000000000000, 0.000000000000
0.851562500000, 1.000000000000, 0.000000000000
0.828125000000, 1.000000000000, 0.000000000000
0.804687500000, 1.000000000000, 0.000000000000
0.781250000000, 1.000000000000, 0.000000000000
0.757812500000, 1.000000000000, 0.000000000000
0.734375000000, 1.000000000000, 0.000000000000
0.710937500000, 1.000000000000, 0.000000000000
0.687500000000, 1.000000000000, 0.000000000000
0.664062500000, 1.000000000000, 0.000000000000
0.640625000000, 1.000000000000, 0.000000000000
0.617187500000, 1.000000000000, 0.000000000000
0.593750000000, 1.000000000000, 0.000000000000
0.570312500000, 1.000000000000, 0.000000000000
0.546875000000, 1.000000000000, 0.000000000000
0.523437500000, 1.000000000000, 0.000000000000
0.500000000000, 1.000000000000, 0.000000000000
0.476562500000, 1.000000000000, 0.000000000000
0.453125000000, 1.000000000000, 0.000000000000
0.429687500000, 1.000000000000, 0.000000000000
0.406250000000, 1.000000000000, 0.000000000000
0.382812500000, 1.000000000000, 0.000000000000
0.359375000000, 1.000000000000, 0.000000000000
0.335937500000, 1.000000000000, 0.000000000000
0.312500000000, 1.000000000000, 0.000000000000
0.289062500000, 1.000000000000, 0.000000000000
0.265625000000, 1.000000000000, 0.000000000000
0.242187500000, 1.000000000000, 0.000000000000
0.218750000000, 1.000000000000, 0.000000000000
0.195312500000, 1.000000000000, 0.000000000000
0.171875000000, 1.000000000000, 0.000000000000
0.148437500000, 1.000000000000, 0.000000000000
0.125000000000, 1.000000000000, 0.000000000000
0.101562500000, 1.000000000000, 0.000000000000
0.078125000000, 1.000000000000, 0.000000000000
0.054687500000, 1.000000000000, 0.000000000000
0.031250000000, 1.000000000000, 0.000000000000
0.007812500000, 1.000000000000, 0.000000000000
0.000000000000, 1.000000000000, 0.015625000000
0.000000000000, 1.000000000000, 0.039062500000
0.000000000000, 1.000000000000, 0.062500000000
0.000000000000, 1.000000000000, 0.085937500000
0.000000000000, 1.000000000000, 0.109375000000
0.000000000000, 1.000000000000, 0.132812500000
0.000000000000, 1.000000000000, 0.156250000000
0.000000000000, 1.000000000000, 0.179687500000
0.000000000000, 1.000000000000, 0.203125000000
0.000000000000, 1.000000000000, 0.226562500000
0.000000000000, 1.000000000000, 0.250000000000
0.000000000000, 1.000000000000, 0.273437500000
0.000000000000, 1.000000000000, 0.296875000000
0.000000000000, 1.000000000000, 0.320312500000
0.000000000000, 1.000000000000, 0.343750000000
0.000000000000, 1.000000000000, 0.367187500000
0.000000000000, 1.000000000000, 0.390625000000
0.000000000000, 1.000000000000, 0.414062500000
0.000000000000, 1.000000000000, 0.437500000000
0.000000000000, 1.000000000000, 0.460937500000
0.000000000000, 1.000000000000, 0.484375000000
0.000000000000, 1.000000000000, 0.507812500000
0.000000000000, 1.000000000000, 0.531250000000
0.000000000000, 1.000000000000, 0.554687500000
0.000000000000, 1.000000000000, 0.578125000000
0.000000000000, 1.000000000000, 0.601562500000
0.000000000000, 1.000000000000, 0.625000000000
0.000000000000, 1.000000000000, 0.648437500000
0.000000000000, 1.000000000000, 0.671875000000
0.000000000000, 1.000000000000, 0.695312500000
0.000000000000, 1.000000000000, 0.718750000000
0.000000000000, 1.000000000000, 0.742187500000
0.000000000000, 1.000000000000, 0.765625000000
0.000000000000, 1.000000000000, 0.789062500000
0.000000000000, 1.000000000000, 0.812500000000
0.000000000000, 1.000000000000, 0.835937500000
0.000000000000, 1.000000000000, 0.859375000000
0.000000000000, 1.000000000000, 0.882812500000
0.000000000000, 1.000000000000, 0.906250000000
0.000000000000, 1.000000000000, 0.929687500000
0.000000000000, 1.000000000000, 0.953125000000
0.000000000000, 1.000000000000, 0.976562500000
0.000000000000, 1.000000000000, 1.000000000000
0.000000000000, 0.976562500000, 1.000000000000
0.000000000000, 0.953125000000, 1.000000000000
0.000000000000, 0.929687500000, 1.000000000000
0.000000000000, 0.906250000000, 1.000000000000
0.000000000000, 0.882812500000, 1.000000000000
0.000000000000, 0.859375000000, 1.000000000000
0.000000000000, 0.835937500000, 1.000000000000
0.000000000000, 0.812500000000, 1.000000000000
0.000000000000, 0.789062500000, 1.000000000000
0.000000000000, 0.765625000000, 1.000000000000
0.000000000000, 0.742187500000, 1.000000000000
0.000000000000, 0.718750000000, 1.000000000000
0.000000000000, 0.695312500000, 1.000000000000
0.000000000000, 0.671875000000, 1.000000000000
0.000000000000, 0.648437500000, 1.000000000000
0.000000000000, 0.625000000000, 1.000000000000
0.000000000000, 0.601562500000, 1.000000000000
0.000000000000, 0.578125000000, 1.000000000000
0.000000000000, 0.554687500000, 1.000000000000
0.000000000000, 0.531250000000, 1.000000000000
0.000000000000, 0.507812500000, 1.000000000000
0.000000000000, 0.484375000000, 1.000000000000
0.000000000000, 0.460937500000, 1.000000000000
0.000000000000, 0.437500000000, 1.000000000000
0.000000000000, 0.414062500000, 1.000000000000
0.000000000000, 0.390625000000, 1.000000000000
0.000000000000, 0.367187500000, 1.000000000000
0.000000000000, 0.343750000000, 1.000000000000
0.000000000000, 0.320312500000, 1.000000000000
0.000000000000, 0.296875000000, 1.000000000000
0.000000000000, 0.273437500000, 1.000000000000
0.000000000000, 0.250000000000, 1.000000000000
0.000000000000, 0.226562500000, 1.000000000000
0.000000000000, 0.203125000000, 1.000000000000
0.000000000000, 0.179687500000, 1.000000000000
0.000000000000, 0.156250000000, 1.000000000000
0.000000000000, 0.132812500000, 1.000000000000
0.000000000000, 0.109375000000, 1.000000000000
0.000000000000, 0.085937500000, 1.000000000000
0.000000000000, 0.062500000000, 1.000000000000
0.000000000000, 0.039062500000, 1.000000000000
0.000000000000, 0.015625000000, 1.000000000000
0.007812500000, 0.000000000000, 1.000000000000
0.031250000000, 0.000000000000, 1.000000000000
0.054687500000, 0.000000000000, 1.000000000000
0.078125000000, 0.000000000000, 1.000000000000
0.101562500000, 0.000000000000, 1.000000000000
0.125000000000, 0.000000000000, 1.000000000000
0.148437500000, 0.000000000000, 1.000000000000
0.171875000000, 0.000000000000, 1.000000000000
0.195312500000, 0.000000000000, 1.000000000000
0.218750000000, 0.000000000000, 1.000000000000
0.242187500000, 0.000000000000, 1.000000000000
0.265625000000, 0.000000000000, 1.000000000000
0.289062500000, 0.000000000000, 1.000000000000
0.312500000000, 0.000000000000, 1.000000000000
0.335937500000, 0.000000000000, 1.000000000000
0.359375000000, 0.000000000000, 1.000000000000
0.382812500000, 0.000000000000, 1.000000000000
0.406250000000, 0.000000000000, 1.000000000000
0.429687500000, 0.000000000000, 1.000000000000
0.453125000000, 0.000000000000, 1.000000000000
0.476562500000, 0.000000000000, 1.000000000000
0.500000000000, 0.000000000000, 1.000000000000
0.523437500000, 0.000000000000, 1.000000000000
0.546875000000, 0.000000000000, 1.000000000000
0.570312500000, 0.000000000000, 1.000000000000
0.593750000000, 0.000000000000, 1.000000000000
0.617187500000, 0.000000000000, 1.000000000000
0.640625000000, 0.000000000000, 1.000000000000
0.664062500000, 0.000000000000, 1.000000000000
0.687500000000, 0.000000000000, 1.000000000000
0.710937500000, 0.000000000000, 1.000000000000
0.734375000000, 0.000000000000, 1.000000000000
0.757812500000, 0.000000000000, 1.000000000000
0.781250000000, 0.000000000000, 1.000000000000
0.804687500000, 0.000000000000, 1.000000000000
0.828125000000, 0.000000000000, 1.000000000000
0.851562500000, 0.000000000000, 1.000000000000
0.875000000000, 0.000000000000, 1.000000000000
0.898437500000, 0.000000000000, 1.000000000000
0.921875000000, 0.000000000000, 1.000000000000
0.945312500000, 0.000000000000, 1.000000000000
0.968750000000, 0.000000000000, 1.000000000000
0.992187500000, 0.000000000000, 1.000000000000
1.000000000000, 0.000000000000, 0.984375000000
1.000000000000, 0.000000000000, 0.960937500000
1.000000000000, 0.000000000000, 0.937500000000
1.000000000000, 0.000000000000, 0.914062500000
1.000000000000, 0.000000000000, 0.890625000000
1.000000000000, 0.000000000000, 0.867187500000
1.000000000000, 0.000000000000, 0.843750000000
1.000000000000, 0.000000000000, 0.820312500000
1.000000000000, 0.000000000000, 0.796875000000
1.000000000000, 0.000000000000, 0.773437500000
1.000000000000, 0.000000000000, 0.750000000000
1.000000000000, 0.000000000000, 0.726562500000
1.000000000000, 0.000000000000, 0.703125000000
1.000000000000, 0.000000000000, 0.679687500000
1.000000000000, 0.000000000000, 0.656250000000
1.000000000000, 0.000000000000, 0.632812500000
1.000000000000, 0.000000000000, 0.609375000000
1.000000000000, 0.000000000000, 0.585937500000
1.000000000000, 0.000000000000, 0.562500000000
1.000000000000, 0.000000000000, 0.539062500000
1.000000000000, 0.000000000000, 0.515625000000
1.000000000000, 0.000000000000, 0.492187500000
1.000000000000, 0.000000000000, 0.468750000000
1.000000000000, 0.000000000000, 0.445312500000
1.000000000000, 0.000000000000, 0.421875000000
1.000000000000, 0.000000000000, 0.398437500000
1.000000000000, 0.000000000000, 0.375000000000
1.000000000000, 0.000000000000, 0.351562500000
1.000000000000, 0.000000000000, 0.328125000000
1.000000000000, 0.000000000000, 0.304687500000
1.000000000000, 0.000000000000, 0.281250000000
1.000000000000, 0.000000000000, 0.257812500000
1.000000000000, 0.000000000000, 0.234375000000
1.000000000000, 0.000000000000, 0.210937500000
1.000000000000, 0.000000000000, 0.187500000000
1.000000000000, 0.000000000000, 0.164062500000
1.000000000000, 0.000000000000, 0.140625000000
1.000000000000, 0.000000000000, 0.117187500000
1.000000000000, 0.000000000000, 0.093750000000
1.000000000000, 0.000000000000, 0.070312500000
1.000000000000, 0.000000000000, 0.046875000000
1.000000000000, 0.000000000000, 0.023437500000];
T_end = 401.000000000000;


end