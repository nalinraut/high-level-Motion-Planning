# Declare a cpp executable

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(dynamic_reconfigure)

#add_executable(${PROJECT_NAME}_node driver controller channel )
add_executable(${PROJECT_NAME}_node driver controller channel-mod )
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_script roboteq_msgs_generate_messages_cpp ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_script ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME driver_node PREFIX "")

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(${PROJECT_NAME}_config dyn-cfg-client)

add_dependencies(${PROJECT_NAME}_config ${PROJECT_NAME}_script roboteq_msgs_generate_messages_cpp ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_config ${PROJECT_NAME}_script ${catkin_LIBRARIES} ${Boost_LIBRARIES})

