add_library(utils OBJECT
    ComputePassPoint.cpp
    DangerFinder.cpp
    FeedbackCollector.cpp
    RobotDealer.cpp
    utils.cpp
    BTRunner.cpp
    BtDebug.cpp
    BBParser.cpp
    Parts.cpp
    Leaf.cpp
    ScopedBB.cpp
    )
add_dependencies(utils
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
