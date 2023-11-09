# go to build folder and launch this command in terminal:
# cmake -G "Visual Studio 16 2019" -A x64 -DMAYA_VERSION=2023 ../CMakeLists.txt

cmake_minimum_required(VERSION 2.6)

project(maya_viewport_collisions_plugin)

set(EXTERNAL_DIR "${CMAKE_SOURCE_DIR}/external")
set(DEVKIT_LOCATION "${CMAKE_SOURCE_DIR}/external/devkitBase")
set(ENV{DEVKIT_LOCATION} "${CMAKE_SOURCE_DIR}/external/devkitBase")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules")
list(APPEND CMAKE_MODULE_PATH "${DEVKIT_LOCATION}/cmake")


# # Function to recursively add subdirectories
# function(add_subdirectories_to_module_path base_dir)
#     # Get all subdirectories
#     file(GLOB children RELATIVE ${base_dir} ${base_dir}/*)
#     foreach(child ${children})
#         if(IS_DIRECTORY ${base_dir}/${child})
#             list(APPEND CMAKE_MODULE_PATH "${base_dir}/${child}")
#         endif()
#     endforeach()
#     # Set the CMAKE_MODULE_PATH variable to the updated list
#     set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" PARENT_SCOPE)
# endfunction()

# # Call the function with the base directory path
# add_subdirectories_to_module_path("${DEVKIT_LOCATION}/cmake")


message(STATUS "===CMAKE_MODULE_PATH is: ${CMAKE_MODULE_PATH}")

include(${DEVKIT_LOCATION}/cmake/pluginEntry.cmake)

add_subdirectory(src)
