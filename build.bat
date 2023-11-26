set VCVARS_LOCATION: C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Auxiliary/Build
call "%VCVARS_LOCATION%/vcvarsall.bat" x64

set DEVKIT_LOCATION="C:/Users/golub/Documents/maya_viewport_collision_plugin/external/devkitBase"
set MAYA_VERSION=2023 
set MAYA_DEVKIT_LOCATION="C:/Users/golub/Documents/maya_viewport_collision_plugin/external/devkitBase"
set MAYA_LOCATION="C:/Program Files/Autodesk/Maya2023"


@REM set Qt5_DIR=%SDKS_LOCATION%/Qt5/lib/cmake
@REM set Qt5_DIR="C:/Users/golub/Documents/maya_viewport_collision_plugin/external/devkitBase/cmake"


cd ./build
cmake -G "Visual Studio 16 2019" -A x64 -DMAYA_VERSION=2023 ../CMakeLists.txt