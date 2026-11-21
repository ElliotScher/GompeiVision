set(MANAGER_SOURCES
        cpp/main.cpp
        cpp/pipeline/PipelineManager.cpp
)

set(WORKER_SOURCES
        cpp/capture/Camera.cpp
        cpp/detector/FiducialDetector.cpp
        cpp/estimator/CameraPoseEstimator.cpp
        cpp/estimator/SingleTagPoseEstimator.cpp
        cpp/estimator/TagAngleCalculator.cpp
        cpp/io/ConfigInterface.cpp
        cpp/io/DatasetRecorder.cpp
        cpp/io/FieldInterface.cpp
        cpp/io/OutputPublisher.cpp
        cpp/pipeline/Pipeline.cpp
        cpp/pipeline/PipelineHelper.cpp
        cpp/pipeline/PipelineProcess.cpp
)

add_executable(GompeiVision ${MANAGER_SOURCES})
add_executable(GompeiVisionProcess ${WORKER_SOURCES})

set_target_properties(GompeiVision GompeiVisionProcess PROPERTIES
        INSTALL_RPATH "$ORIGIN/../lib"
)

target_include_directories(GompeiVision PUBLIC
        "${CMAKE_SOURCE_DIR}/src/include"
)
target_include_directories(GompeiVisionProcess PUBLIC
        "${CMAKE_SOURCE_DIR}/src/include"
)

target_link_libraries(GompeiVision PRIVATE
        third_party_libs
)
target_link_libraries(GompeiVisionProcess PRIVATE
        third_party_libs
)

# Recorded datasets note which AprilTag the detections came from.
find_package(apriltag QUIET)
target_compile_definitions(GompeiVisionProcess PRIVATE
        GOMPEI_APRILTAG_VERSION="${apriltag_VERSION}"
)

install(TARGETS GompeiVision GompeiVisionProcess DESTINATION bin)

add_custom_command(
        TARGET GompeiVision
        POST_BUILD
        COMMAND ${CMAKE_CPACK_COMMAND}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Packaging project into a .deb file..."
        VERBATIM
)
# Opt-in checks of the estimators against synthetic truth: -DGOMPEI_TESTS=ON
option(GOMPEI_TESTS "Build the CPU pipeline's truth tests" OFF)
if(GOMPEI_TESTS)
    add_executable(PoseEstimatorTruthTest
            tests/PoseEstimatorTruthTest.cpp
            cpp/estimator/CameraPoseEstimator.cpp
            cpp/estimator/SingleTagPoseEstimator.cpp
    )
    target_include_directories(PoseEstimatorTruthTest PRIVATE "${CMAKE_SOURCE_DIR}/src/include")
    target_link_libraries(PoseEstimatorTruthTest PRIVATE third_party_libs)

    # The FPGA project's golden PnP model must match OpenCV's IPPE.
    add_executable(GoldenPnpCrossCheck
            tests/GoldenPnpCrossCheck.cpp
            ${CMAKE_SOURCE_DIR}/model/src/pnp.cpp
    )
    target_include_directories(GoldenPnpCrossCheck PRIVATE
            "${CMAKE_SOURCE_DIR}/src/include" "${CMAKE_SOURCE_DIR}/model/include")
    target_link_libraries(GoldenPnpCrossCheck PRIVATE third_party_libs)
endif()
