add_subdirectory(json)

add_library(alltrees OBJECT
    alltrees.cpp
    alltrees_factory.cpp
    # $<TARGET_OBJECTS:treegen>
    # $<TARGET_OBJECTS:skills>
    # $<TARGET_OBJECTS:conditions>
    # $<TARGET_OBJECTS:tactics>
    )
add_dependencies(alltrees
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    generate_json_t
    )

# target_link_libraries(alltrees
    # $<TARGET_OBJECTS:treegen>
    # $<TARGET_OBJECTS:skills>
    # $<TARGET_OBJECTS:conditions>
    # $<TARGET_OBJECTS:tactics>
    # )

set(JSON_FILES
    ${JSON_FILES}
    PARENT_SCOPE
    )
