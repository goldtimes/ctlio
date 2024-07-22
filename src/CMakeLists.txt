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