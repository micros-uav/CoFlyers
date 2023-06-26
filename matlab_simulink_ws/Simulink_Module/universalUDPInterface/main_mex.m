%%
ipath = ['-I',pwd];
mex(ipath,'UniversalUDPInterface.cpp','UDPInterfaceBase.cpp','UDPInterfacePack.cpp','data_description.cpp');
mex(ipath,'UniversalUDPInterface2.cpp','UDPInterfaceBase.cpp','UDPInterfacePack.cpp','data_description.cpp');
mex(ipath,'UniversalUDPInterface3.cpp','UDPInterfaceBase.cpp','UDPInterfacePack.cpp','data_description.cpp');
mex(ipath,'UniversalUDPInterface4.cpp','UDPInterfaceBase.cpp','UDPInterfacePack.cpp','data_description.cpp');