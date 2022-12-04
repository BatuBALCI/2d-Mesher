cmake_minimum_required(VERSION 3.4.0)

project(Mesh)

add_executable(${PROJECT_NAME} EntryPoint.cpp)

add_subdirectory(DCEL)

target_link_directories(${PROJECT_NAME} PUBLIC DCEL)
target_include_directories(${PROJECT_NAME} PUBLIC DCEL)
target_link_libraries(${PROJECT_NAME} DCEL_Lib)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})