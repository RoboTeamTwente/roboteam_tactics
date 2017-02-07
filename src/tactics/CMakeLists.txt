set(TACTIC_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/AttackerTactic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BasicAttackTactic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BasicDefenseTactic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GoToSideTactic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/DemoTactic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BasicKeeperTactic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/PassToTactic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SecondaryKeeperTactic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/InterceptorsTactic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FreeKickDefenceTactic.cpp
    )
set(TACTIC_SOURCES
    ${TACTIC_SOURCES}
    PARENT_SCOPE
    )

add_library(tactics OBJECT
    ${TACTIC_SOURCES}
    )
add_dependencies(tactics
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
