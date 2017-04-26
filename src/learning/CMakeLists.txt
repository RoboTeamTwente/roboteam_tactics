add_library(roboteam_tactics_learning
    LearnGoToPos.cpp
    )
add_library(learning OBJECT
    LearnGoToPos.cpp
    )
add_dependencies(roboteam_tactics_learning
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
add_dependencies(learning
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
