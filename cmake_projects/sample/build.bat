cd build
cmake -A win32 -MAKE_BUILD_TYPE=Release ..
"C:\Program Files (x86)\MSBuild\14.0\Bin\amd64\MSBuild.exe" ALL_BUILD.vcxproj /maxcpucount:8 /p:CL_MPCount=8 /p:Configuration=Release /p:Platform="win32" /p:PreferredToolArchitecture="win32"

@echo off
cd ..
cd bin
cd Release
move *.* ..\
cd ..
rmdir Release
cd ..