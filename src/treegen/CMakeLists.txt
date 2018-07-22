add_library(treegen OBJECT
    LeafRegister.cpp
    NodeFactory.cpp
    )

set_property(TARGET treegen PROPERTY POSITION_INDEPENDENT_CODE ON)

add_dependencies(treegen
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
