# If you want to add a tactic, please do that in unity.cpp

set(TACTIC_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/unity.cpp
    )
set(TACTIC_SOURCES
    ${TACTIC_SOURCES}
    PARENT_SCOPE
    )

add_library(tactics OBJECT
    ${TACTIC_SOURCES}
    )

set_property(TARGET tactics PROPERTY POSITION_INDEPENDENT_CODE ON)

add_dependencies(tactics
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
