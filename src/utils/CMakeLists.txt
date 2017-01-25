add_library(utils OBJECT
    Cone.cpp
    ComputePassPoint.cpp
    DangerFinder.cpp
    Draw.cpp
    FeedbackCollector.cpp
    LastRef.cpp
    Math.cpp
    RobotDealer.cpp
    utils.cpp
    BTRunner.cpp
    BtDebug.cpp
    )
add_dependencies(utils
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
