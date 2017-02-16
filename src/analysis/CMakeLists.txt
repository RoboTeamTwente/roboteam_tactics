add_library(roboteam_tactics_analysis
    SimpleAnalysis.cpp
    SpaceAnalysis.cpp
    )
add_library(analysis OBJECT
    SimpleAnalysis.cpp
    SpaceAnalysis.cpp
    )
add_dependencies(roboteam_tactics_analysis
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
add_dependencies(analysis
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
