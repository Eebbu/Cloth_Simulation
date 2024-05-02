cmake_minimum_required(VERSION 3.10)
project(main)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/includes
    ${PROJECT_SOURCE_DIR}/lib)

link_libraries(${PROJECT_SOURCE_DIR}/lib/libglfw.3.dylib)

add_executable(${PROJECT_NAME} 
    src/glad.c
    src/main.cpp)

if (APPLE)
    target_link_libraries(${PROJECT_NAME} "-framework OpenGL")
endif()