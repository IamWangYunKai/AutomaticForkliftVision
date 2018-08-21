@echo off
for /f "delims=" %%a in ('dir /b/a-d/oN *.proto') do (
start protoc.exe --cpp_out=./ %%a
)
