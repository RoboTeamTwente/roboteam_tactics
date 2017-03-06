add_library(roboteam_tactics_rules
    Rules.cpp
    HaltRule.cpp
    )
add_library(rules OBJECT
    Rules.cpp
    HaltRule.cpp
    )
add_dependencies(roboteam_tactics_rules
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
add_dependencies(rules
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
