# go to build folder and launch this command in terminal:
# cmake -G "Visual Studio 16 2019" -A x64 -DMAYA_VERSION=2023 -DMAYA_DEVKIT_LOCATION="C:/Users/golub/Documents/maya_viewport_collision_plugin/external/devkitBase" -DMAYA_LOCATION="C:/Program Files/Autodesk/Maya2023" ../CMakeLists.txt

cmake_minimum_required(VERSION 2.6)

project(maya_viewport_collisions_plugin)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/modules)

add_subdirectory(src)
