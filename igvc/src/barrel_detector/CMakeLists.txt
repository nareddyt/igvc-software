add_executable(barrel_detector main.cpp barreldetector.cpp)
target_link_libraries(barrel_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES})
