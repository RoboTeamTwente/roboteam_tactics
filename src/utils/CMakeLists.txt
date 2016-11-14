add_library(utils
    Cone.cpp
    DangerFinder.cpp
    FeedbackCollector.cpp
    LastWorld.cpp
    Math.cpp
    RobotDealer.cpp
    utils.cpp
    NodeFactory.cpp
    BTRunner.cpp
    )
add_dependencies(utils
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
target_link_libraries(utils skills conditions alltrees)
