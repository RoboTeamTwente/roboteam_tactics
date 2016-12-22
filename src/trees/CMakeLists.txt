add_subdirectory(json)

add_library(alltrees OBJECT
    alltrees.cpp
    )
add_dependencies(alltrees
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    generate_json_t
    )

set(JSON_FILES
    ${JSON_FILES}
    PARENT_SCOPE
    )

set(JSON_PROJECTS
    ${JSON_PROJECTS}
    PARENT_SCOPE
    )
