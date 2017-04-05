add_library(treegen OBJECT
    LeafRegister.cpp
    NodeFactory.cpp
    )
add_dependencies(treegen
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
