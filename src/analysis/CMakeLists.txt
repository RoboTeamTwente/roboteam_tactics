add_library(roboteam_tactics_analysis
    SimpleAnalysis.cpp
    )
add_dependencies(roboteam_tactics_analysis
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
