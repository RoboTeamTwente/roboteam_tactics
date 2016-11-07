add_library(utils
    Cone.cpp
    DangerFinder.cpp
    FeedbackCollector.cpp
    LastWorld.cpp
    utils.cpp
    )
add_dependencies(utils
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
target_link_libraries(utils skills conditions)
