
add_library(utils OBJECT
    BBParser.cpp
    BTRunner.cpp
    BtDebug.cpp
    ComputePassPoint.cpp
    Control.cpp
    FeedbackCollector.cpp
    Leaf.cpp
    Parts.cpp
    RobotDealer.cpp
    ScopedBB.cpp
    debug_print.cpp
    utils.cpp
    RefStateSwitch.cpp
    StrategyComposer.cpp
    GoalPartition.cpp
    Intercept.cpp
    intercept/intercept2.cpp
    )
add_dependencies(utils
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

