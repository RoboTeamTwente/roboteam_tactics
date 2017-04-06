set(BT_SOURCES
    composites/ParallelSequence.cpp # Check
    composites/MemSequence.cpp      # Check
    composites/Selector.cpp         # Check
    composites/MemSelector.cpp      # Check
    composites/Sequence.cpp         # Check
    decorators/UntilSuccess.cpp     # Check
    decorators/Failer.cpp           # Check
    decorators/Inverter.cpp         # Check
    decorators/Repeater.cpp         # Check
    decorators/UntilFail.cpp        # Check
    decorators/Succeeder.cpp        # Check
    Decorator.cpp                   # Check
    Composite.cpp                   # Check
    Leaf.cpp                        # Check
    BehaviorTree.cpp                # Check
    Blackboard.cpp                  # Check
    Node.cpp                        # Check
    )

add_library(bt
    ${BT_SOURCES}
    )
add_dependencies(bt
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )


