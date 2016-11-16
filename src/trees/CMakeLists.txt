add_subdirectory(json)

add_library(alltrees
    alltrees.cpp
    alltrees_factory.cpp
    )
add_dependencies(alltrees
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    generate_json_t
    )

target_link_libraries(alltrees
    skills
    conditions
    tactics
    )

set(JSON_FILES
    ${JSON_FILES}
    PARENT_SCOPE
    )
