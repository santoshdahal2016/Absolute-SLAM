add_subdirectory(openvslam)

add_subdirectory(render)


if(USE_PANGOLIN_VIEWER)
    add_subdirectory(pangolin_viewer)
endif()

if(USE_SOCKET_PUBLISHER)
    add_subdirectory(socket_publisher)
endif()

