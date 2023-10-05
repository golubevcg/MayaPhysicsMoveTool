cmake_minimum_required(VERSION 2.6)

project(maya_viewport_collisions_plugin)

# Set a variable for the external directory
set(EXTERNAL_DIR "${CMAKE_SOURCE_DIR}/external")

# Add Boost library directory to the linker search directories
link_directories("${EXTERNAL_DIR}/boost_1_83_0/libs")

# Add ReactPhysics3D as a subdirectory with a binary directory
add_subdirectory(${EXTERNAL_DIR}/reactphysics3d-master ${CMAKE_BINARY_DIR}/reactphysics3d-master)

file(GLOB_RECURSE MY_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
message("My sources: ${MY_SOURCES}")  # Print out the found source files

set(SOURCE_FILES ${MY_SOURCES})

find_package(Maya REQUIRED)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

# Add Boost and ReactPhysics3D include directories
target_include_directories(${PROJECT_NAME} 
    PRIVATE 
        Maya::Maya
        "${EXTERNAL_DIR}/boost_1_83_0"
        "${EXTERNAL_DIR}/reactphysics3d-master/src"
    PUBLIC 
        "${CMAKE_CURRENT_BINARY_DIR}" 
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

# Link against ReactPhysics3D
target_link_libraries(${PROJECT_NAME} PRIVATE Maya::Maya reactphysics3d)

MAYA_PLUGIN(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${MAYA_TARGET_TYPE} DESTINATION plug-ins/${MAYA_VERSION})
