# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/string_manip.h
               ${CMAKE_CURRENT_SOURCE_DIR}/transform_interface.h
               ${CMAKE_CURRENT_SOURCE_DIR}/transform.h
               ${CMAKE_CURRENT_SOURCE_DIR}/image.h
               ${CMAKE_CURRENT_SOURCE_DIR}/typedefs.h
               ${CMAKE_CURRENT_SOURCE_DIR}/pose.h
               ${CMAKE_CURRENT_SOURCE_DIR}/trajectory.h
               ${CMAKE_CURRENT_SOURCE_DIR}/filesystem.h
               ${CMAKE_CURRENT_SOURCE_DIR}/result_logger.h

               ${CMAKE_CURRENT_SOURCE_DIR}/string_manip.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/transform.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/image.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/typedefs.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/pose.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/trajectory.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/filesystem.cpp
               ${CMAKE_CURRENT_SOURCE_DIR}/result_logger.cpp

               )

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/absolute)
