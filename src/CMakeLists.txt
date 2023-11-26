cmake_minimum_required(VERSION 2.6)

project(MayaViewportCollisionsPlugin)

# Set a variable for the external directory
set(EXTERNAL_DIR "${CMAKE_SOURCE_DIR}/external")
set(BULLET_ROOT "${EXTERNAL_DIR}/bullet3")
add_subdirectory("${BULLET_ROOT}" "${CMAKE_BINARY_DIR}/bullet3")

# Add Boost library directory to the linker search directories
link_directories("${EXTERNAL_DIR}/boost_1_83_0/libs")
link_directories("${EXTERNAL_DIR}/devkit/lib")
link_directories("${EXTERNAL_DIR}/devkit/lib")

# Add ReactPhysics3D as a subdirectory with a binary directory

file(GLOB_RECURSE MY_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
message("My sources: ${MY_SOURCES}")  # Print out the found source files

# set(SOURCE_FILES ${MY_SOURCES})
file(GLOB_RECURSE MY_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB_RECURSE MY_HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

list(APPEND MY_SOURCES ${MY_HEADERS})
set(SOURCE_FILES ${MY_SOURCES})

include("${EXTERNAL_DIR}/devkitBase/cmake/pluginEntry.cmake")

build_plugin()

find_package(Maya REQUIRED)

# add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

# Add Boost and ReactPhysics3D include directories
target_include_directories(${PROJECT_NAME} 
    PRIVATE 
        Maya::Maya
        "${EXTERNAL_DIR}/devkitBase/include"
        "${EXTERNAL_DIR}/devkitBase/devkit/bin"
        "${EXTERNAL_DIR}/boost_1_83_0"
        "${BULLET_ROOT}/src"
    PUBLIC 
        "${CMAKE_CURRENT_BINARY_DIR}" 
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

# Link against ReactPhysics3D
target_link_libraries(${PROJECT_NAME} PRIVATE 
    Maya::Maya
    BulletDynamics 
    BulletCollision 
    LinearMath
    )

MAYA_PLUGIN(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${MAYA_TARGET_TYPE} DESTINATION plug-ins/${MAYA_VERSION})

get_target_property(COMP_FLAGS ${PROJECT_NAME} COMPILE_FLAGS)
get_target_property(LINK_FLAGS ${PROJECT_NAME} LINK_FLAGS)
message(STATUS "Compile flags: ${COMP_FLAGS}")
message(STATUS "Linker flags: ${LINK_FLAGS}")



set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MT")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")