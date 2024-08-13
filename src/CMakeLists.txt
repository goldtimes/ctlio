add_executable(main_node
    app/main.cc
    lidar_process/lidar_process.cc
    lio/lidar_odom.cc
    lio/lio_utils.cc
    ekf_utils/static_imu_init.cc
    tools/timer/timer.cc
    factor/poseParameterization.cc
    factor/factors.cc
)

target_link_libraries(
    main_node
    ${third_party_libs}
    ${catkin_LIBRARIES}
)

# add_executable(test_feature_extractor 
#         test/feature_extract.cc
#         lidar_process/segment/segment.cc
#         lidar_process/segment/pointsCorrect.cc
#         lidar_process/lidar_feature/lidar_feature_extractor.cc
# )

# target_link_libraries(
#     test_feature_extractor
#     ${third_party_libs}
#     ${catkin_LIBRARIES}
# )