# Add sources
target_sources(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/frame_publisher.h
        ${CMAKE_CURRENT_SOURCE_DIR}/map_publisher.h
        ${CMAKE_CURRENT_SOURCE_DIR}/frame_publisher.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/map_publisher.cc)

if(USE_ZMQ_LINE_PUBLISHER)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ZMQ REQUIRED libzmq)

    # cppzmq is header-only; find it manually
    find_path(CPPZMQ_INCLUDE_DIR zmq.hpp
        HINTS /usr/include /usr/local/include)

    if(NOT CPPZMQ_INCLUDE_DIR)
        message(FATAL_ERROR "cppzmq header (zmq.hpp) not found — install cppzmq-dev")
    endif()

    target_sources(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/line_zmq_publisher.h
        ${CMAKE_CURRENT_SOURCE_DIR}/line_zmq_publisher.cc)

    target_include_directories(${PROJECT_NAME} PUBLIC ${CPPZMQ_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${ZMQ_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_ZMQ_LINE_PUBLISHER)

    message(STATUS "ZMQ line publisher: ENABLED (endpoint default tcp://*:5557)")
else()
    message(STATUS "ZMQ line publisher: DISABLED")
endif()

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${PLPSLAM_INCLUDE_INSTALL_DIR}/publish)
