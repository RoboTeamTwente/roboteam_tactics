add_library(utils OBJECT
    BBParser.cpp
    BTRunner.cpp
    BtDebug.cpp
    ComputePassPoint.cpp
    DangerFinder.cpp
    FeedbackCollector.cpp
    Leaf.cpp
    Parts.cpp
    RobotDealer.cpp
    ScopedBB.cpp
    debug_print.cpp
    utils.cpp
    )
add_dependencies(utils
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
