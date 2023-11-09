cmake_minimum_required(VERSION 3.12)
project(MayaViewportCollisionsPlugin)


set(BULLET_ROOT "${EXTERNAL_DIR}/bullet3")
add_subdirectory("${BULLET_ROOT}" "${CMAKE_BINARY_DIR}/bullet3")

link_directories("${EXTERNAL_DIR}/boost_1_83_0/libs")

include(${DEVKIT_LOCATION}/cmake/pluginEntry.cmake)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

file(GLOB_RECURSE MY_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
message("My sources: ${MY_SOURCES}")

file(GLOB_RECURSE MY_HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
message("My headers: ${MY_HEADERS}")

set(HEADER_FILES ${MY_HEADERS})
set(SOURCE_FILES ${MY_SOURCES})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake") # Adjust the path to where FindMaya.cmake and FindMaya_Qt.cmake are located
# include(FindMaya)
# include(FindMaya_Qt)

find_package(Maya REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets Gui OpenGL REQUIRED)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})

target_include_directories(${PROJECT_NAME} 
    PRIVATE 
        ${MAYA_INCLUDE_DIR}
        "${EXTERNAL_DIR}/boost_1_83_0"
        "${BULLET_ROOT}/src"
    PUBLIC 
        "${CMAKE_CURRENT_BINARY_DIR}" 
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(${PROJECT_NAME} PRIVATE 
    ${MAYA_LIBRARIES} 
    Qt5::Core 
    Qt5::Widgets 
    Qt5::Gui 
    Qt5::OpenGL 
    BulletDynamics 
    BulletCollision 
    LinearMath
    )

# Your custom MAYA_PLUGIN function/macro - make sure it is defined before this call
MAYA_PLUGIN(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${MAYA_TARGET_TYPE} DESTINATION plug-ins/${MAYA_VERSION})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MT")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
