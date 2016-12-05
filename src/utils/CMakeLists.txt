add_library(utils OBJECT
    Cone.cpp
    DangerFinder.cpp
    FeedbackCollector.cpp
    LastWorld.cpp
    LastRef.cpp
    Math.cpp
    RobotDealer.cpp
    utils.cpp
    BTRunner.cpp
    # $<TARGET_OBJECTS:skills>
    # $<TARGET_OBJECTS:conditions>
    )
add_dependencies(utils
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
# target_link_libraries(utils
    # $<TARGET_OBJECTS:skills>
    # $<TARGET_OBJECTS:conditions>
    # alltrees
    # )
