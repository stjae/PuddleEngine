cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME shaderCompiler)
project(${PROJECT_NAME})

find_package(Vulkan REQUIRED)

set(GLSL_VALIDATOR "$ENV{VULKAN_SDK}/Bin/glslc")

file(GLOB GLSL_SOURCE_FILES
        "shader/*.frag"
        "shader/*.vert"
)

foreach (GLSL ${GLSL_SOURCE_FILES})
    get_filename_component(FILE_NAME ${GLSL} NAME)
    set(SPIRV "${CMAKE_CURRENT_SOURCE_DIR}/shader/spv/${FILE_NAME}.spv")
    add_custom_command(
            OUTPUT ${SPIRV}
            COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_SOURCE_DIR}/shader/spv/"
            COMMAND ${GLSL_VALIDATOR} ${GLSL} -o ${SPIRV}
            DEPENDS ${GLSL})
    list(APPEND SPIRV_BINARY_FILES ${SPIRV})
endforeach (GLSL)

add_custom_target(shaderFiles ALL DEPENDS ${SPIRV_BINARY_FILES})
add_executable(${PROJECT_NAME} shader.cpp)
add_dependencies(${PROJECT_NAME} shaderFiles)