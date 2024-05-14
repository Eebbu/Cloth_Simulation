cmake_minimum_required(VERSION 3.10)
project(research VERSION 1.0)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)

if (APPLE)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
    include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/includes
        ${PROJECT_SOURCE_DIR}/lib)
    link_libraries(${PROJECT_SOURCE_DIR}/lib/libglfw.3.dylib)

    # Executable to be built from the specified source files
    add_executable(${PROJECT_NAME}
        src/glad.c
        src/main.cpp
        src/stb_image.cpp
    )
    target_link_libraries(${PROJECT_NAME} "-framework OpenGL")
endif()

if(WIN32)
    add_executable(${PROJECT_NAME}
        src/glad.c
        src/main.cpp
        src/stb_image.cpp
    )
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/lib/libglad.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/lib/libglfw3.a)
    target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/lib/libglfw3dll.a)
    include_directories(${PROJECT_SOURCE_DIR}/includes ${PROJECT_SOURCE_DIR}/lib)
endif()