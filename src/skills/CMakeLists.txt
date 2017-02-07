set(SKILL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/AimAt.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/AvoidRobots.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GetBall.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Kick.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/RotateAroundPoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Block.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FollowPath.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GoToPos.cpp
    
    
    ${CMAKE_CURRENT_SOURCE_DIR}/Dribble.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/KeeperBlock.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SecondaryKeeper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/StandFree.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/RandomDrive.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NaiveBlockGoal.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ShootAtGoal.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/Sleep.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BackUp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParamSet.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Chip.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Failer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Runner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/DefendGoalarea.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Harass.cpp

    
    )
set(SKILL_SOURCES
    ${SKILL_SOURCES}
    PARENT_SCOPE
    )

add_library(skills OBJECT
    ${SKILL_SOURCES}
    )
add_dependencies(skills
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
