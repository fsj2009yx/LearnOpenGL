@echo off
setlocal

REM 若需要公共 CMake 参数，可在这里设置
REM set COMMON_CMAKE_CONFIG_PARAMS=-DSOME_OPTION=ON

mkdir out
cd out

cmake -G "Visual Studio 17 2022" %COMMON_CMAKE_CONFIG_PARAMS% ..\

cmake --build . --config Debug

endlocal
