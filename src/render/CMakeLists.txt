# ----- Create Render library -----
add_library(render
        SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/camera.h
        ${CMAKE_CURRENT_SOURCE_DIR}/mesh.h
        ${CMAKE_CURRENT_SOURCE_DIR}/model.h
        ${CMAKE_CURRENT_SOURCE_DIR}/shader.h
        ${CMAKE_CURRENT_SOURCE_DIR}/shader_m.h
        ${CMAKE_CURRENT_SOURCE_DIR}/shader_s.h)

# Set output directory of the library
set_target_properties(render PROPERTIES
        OUTPUT_NAME render
        LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib LINKER_LANGUAGE CXX)

