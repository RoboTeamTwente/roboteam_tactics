set(TACTIC_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/BasicAttackTactic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/BasicDefenseTactic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/GoToSideTactic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/DemoTactic.cpp
    )
set(TACTIC_SOURCES
    ${TACTIC_SOURCES}
    PARENT_SCOPE
    )

add_library(tactics
    ${TACTIC_SOURCES}
    )
add_dependencies(tactics
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
