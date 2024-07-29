add_executable(${PROJECT_NAME}_node 
    app/main.cc
    lidar_process/lidar_process.cc
    tools/timer/timer.cc
)

target_link_libraries(
    ${PROJECT_NAME}_node
    ${third_party_libs}
    ${catkin_LIBRARIES}
)

add_executable(test_feature_extractor 
        test/feature_extract.cc
        lidar_process/segment/segment.cc
        lidar_process/segment/pointsCorrect.cc
        lidar_process/lidar_feature/lidar_feature_extractor.cc
)

target_link_libraries(
    test_feature_extractor
    ${third_party_libs}
    ${catkin_LIBRARIES}
)