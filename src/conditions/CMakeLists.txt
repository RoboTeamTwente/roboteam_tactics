set(CONDITION_SOURCES
    
    ${CMAKE_CURRENT_SOURCE_DIR}/CanSeeRobot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/DistanceXToY.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/IHaveBall.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BallOnOurSide.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CanInterceptBall.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CanReachPoint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CanSeePoint.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/CanSeeTheirGoal.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/CanPassSafely.cpp
    
    ${CMAKE_CURRENT_SOURCE_DIR}/IsBallInGoal.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/TeamHasBall.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/TheyHaveBall.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/WeHaveBall.cpp
    
    ${CMAKE_CURRENT_SOURCE_DIR}/IsRefStage.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/IsRefCommand.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/IsBallInZone.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ParamCheck.cpp
    
    )
set(CONDITION_SOURCES
    ${CONDITION_SOURCES}
    PARENT_SCOPE
    )

add_custom_target(CONDITION_SOURCES_TARGET
    DEPENDS
        ${CONDITION_SOURCES}
    )

add_library(conditions OBJECT
    ${CONDITION_SOURCES}
    )
add_dependencies(conditions
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
