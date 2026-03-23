@echo off
setlocal
call "%ProgramFiles%\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -host_arch=amd64 -arch=arm64
msbuild "%~dp0goodix9916r.sln" /m /t:Build /p:Configuration=Release /p:Platform=ARM64 /p:GenerateFullPaths=true /v:m
exit /b %errorlevel%
