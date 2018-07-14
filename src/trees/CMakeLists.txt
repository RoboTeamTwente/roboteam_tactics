add_subdirectory(projects)

foreach(json_project ${JSON_PROJECTS})

    get_filename_component(project_basename ${json_project} NAME_WE)

    set(project_sources
        ${project_sources}
        ${project_basename}_trees.cpp
        )

    set(project_targets
        ${project_targets}
        generate_json_project_${project_basename}_t
        )

    MESSAGE("-- Added project '" ${BLUE_TEXT} ${BOLD_TEXT} ${project_basename} ${RESET_TEXT} "' to sources & targets.")

endforeach(json_project)

add_library(alltrees OBJECT
#    ${PROJECT_SOURCE_DIR}/src/trees/alltrees.cpp
    ${project_sources}
    )
add_dependencies(alltrees
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    generate_json_t
    ${project_targets}
    )

set(JSON_FILES
    ${JSON_FILES}
    PARENT_SCOPE
    )

set(JSON_PROJECTS
    ${JSON_PROJECTS}
    PARENT_SCOPE
    )
