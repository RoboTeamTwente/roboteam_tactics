cmake_minimum_required(VERSION 2.8.3)
project(roboteam_tactics)

find_package(catkin REQUIRED COMPONENTS
  roboteam_msgs
  roboteam_utils
  roscpp
  std_msgs
  actionlib_msgs
  actionlib
)

set(CMAKE_CXX_FLAGS "\
--std=c++14 ${CMAKE_CXX_FLAGS} \
-Wall \
-Wno-unused-function \
-Wsign-compare \
")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES roboteam_tactics
#  CATKIN_DEPENDS roboteam_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

file(GLOB TACTICS_CONDITIONS_SOURCES "src/conditions/*.cpp")
file(GLOB TACTICS_SKILLS_SOURCES "src/skills/*.cpp")
file(GLOB TACTICS_TACTICS_SOURCES "src/tactics/*.cpp")

# To signify that the file is generated s.t. cmake won't throw
# an error if it can't find the file in the config step.
#set_source_files_properties(src/trees/alltrees.cpp
#                            PROPERTIES GENERATED TRUE)

add_executable(brain
    src/main.cpp
    src/LastWorld.cpp
    src/PickedTactic.cpp
    src/trees/alltrees.cpp
    ${TACTICS_CONDITIONS_SOURCES}
    ${TACTICS_SKILLS_SOURCES}
    ${TACTICS_TACTICS_SOURCES}
)
add_dependencies(brain ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} generate_json_t)
target_link_libraries(brain ${catkin_LIBRARIES})

add_executable(KickAtGoal
    src/role_tests/KickAtGoal.cpp
    src/Aggregator.cpp
    src/LastWorld.cpp
    src/PickedTactic.cpp
    src/trees/alltrees.cpp
    ${TACTICS_CONDITIONS_SOURCES}
    ${TACTICS_SKILLS_SOURCES}
    ${TACTICS_TACTICS_SOURCES}
)
add_dependencies(KickAtGoal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} generate_json_t)
target_link_libraries(KickAtGoal ${catkin_LIBRARIES})

add_executable(Keeper
    src/role_tests/Keeper.cpp
    src/Aggregator.cpp
    src/LastWorld.cpp
    src/PickedTactic.cpp
    src/trees/alltrees.cpp
    ${TACTICS_CONDITIONS_SOURCES}
    ${TACTICS_SKILLS_SOURCES}
    ${TACTICS_TACTICS_SOURCES}
)
add_dependencies(Keeper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} generate_json_t)
target_link_libraries(Keeper ${catkin_LIBRARIES})

# Executable that can convert behavior3 json to C++ functions
add_executable(converter
    src/treegen/BTBuilder.cpp
    src/treegen/converter.cpp
    )
add_dependencies(converter generate_leafs_t)

add_executable(TestX
    src/skills/tests/TestX.cpp
    src/LastWorld.cpp
    ${TACTICS_CONDITIONS_SOURCES}
    ${TACTICS_SKILLS_SOURCES}
    )
add_dependencies(TestX ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} generate_leafs_t)
target_link_libraries(TestX ${catkin_LIBRARIES})

############################################################
#### Stupid stuff to get the json files to be converted ####
############################################################

# Custom target will always cause its dependencies to be evaluated and is
# run by default
#add_custom_target(dummy_target ALL
    #DEPENDS
        #generate_json
        #generate_leafs
    #)

# custom_output will always be rebuilt because it depends on always_rebuild
add_custom_target(generate_json_t DEPENDS generate_json)
add_custom_command(
    OUTPUT generate_json
    COMMAND ${PROJECT_SOURCE_DIR}/scripts/generate_cpp_from_json.sh
    DEPENDS
        always_rebuild
        converter
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Generating C++ from JSON files"
    )

# This extra custom target is needed to use it in brain
# as a dependency. (Something to do with working folders)
# Generates files (like allconditions.h and allconditions_set.h)
# that can be used to see which conditions exist (using the set)
# and include all possible conditions at once.
add_custom_target(generate_leafs_t DEPENDS generate_leafs)
add_custom_command(
    OUTPUT generate_leafs
    COMMAND ${PROJECT_SOURCE_DIR}/scripts/generate_all_leafs.sh
    DEPENDS
        always_rebuild
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Generating C++ based on implemented skills, conditions, and tactics"
    )

# Dummy output which is never actually produced. Anything that depends on
# this will always be rebuilt.
add_custom_command(
    OUTPUT always_rebuild
    COMMAND cmake -E echo
    )

#############################
#### End of stupid stuff ####
#############################

# Hooks that delete all generated files
file(GLOB GENERATED_FILES
    "include/roboteam_tactics/generated/*.h"
    )

set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES ${GENERATED_FILES})
# End of hooks

#add_executable(GetBall
#	src/skills/GetBall.cpp
#    src/skills/GoToPos.cpp
#    src/Aggregator.cpp
#    src/LastWorld.cpp
#)
#add_dependencies(GetBall ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(GetBall ${catkin_LIBRARIES})

add_executable(AimSkill
	src/AimSkill.cpp
    src/Aggregator.cpp
    src/LastWorld.cpp
)
add_dependencies(AimSkill ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(AimSkill ${catkin_LIBRARIES})


add_executable(RotateAroundPoint
	src/skills/tests/RotateAroundPointTest.cpp
	src/skills/RotateAroundPoint.cpp
    src/Aggregator.cpp
    src/LastWorld.cpp
)
add_dependencies(RotateAroundPoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(RotateAroundPoint ${catkin_LIBRARIES})

add_executable(GoToPosTest
	src/skills/tests/GoToPosTest.cpp
    src/skills/GoToPos.cpp
    src/Aggregator.cpp
    src/LastWorld.cpp
)
add_dependencies(GoToPosTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(GoToPosTest ${catkin_LIBRARIES})

add_executable(GetBallTest
    src/skills/tests/GetBallTest.cpp
    src/skills/GetBall.cpp
    src/skills/GoToPos.cpp
    src/conditions/IHaveBall.cpp
    src/Aggregator.cpp
    src/LastWorld.cpp
)
add_dependencies(GetBallTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(GetBallTest ${catkin_LIBRARIES})

add_executable(KickTest
    src/skills/tests/KickTest.cpp
    src/skills/Kick.cpp
    src/Aggregator.cpp
    src/LastWorld.cpp
)

add_executable(IcptTest
    src/conditions/CanInterceptBall.cpp
    src/LastWorld.cpp
)
add_dependencies(IcptTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(IcptTest ${catkin_LIBRARIES})
target_compile_options(IcptTest PRIVATE "-DDEBUG_ICPT")

catkin_add_gtest(VerificationTest
    test/main.cpp
    test/VerificationTest.cpp
)
target_link_libraries(VerificationTest ${catkin_LIBRARIES})

catkin_add_gtest(ConditionsTest
    test/main.cpp
    test/ConditionsTest.cpp

    src/conditions/IHaveBall.cpp
    src/LastWorld.cpp
)
target_link_libraries(ConditionsTest ${catkin_LIBRARIES})

add_dependencies(KickTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(KickTest ${catkin_LIBRARIES})

add_executable(FollowPathTest
    src/skills/tests/FollowPathTest.cpp
    src/skills/FollowPath.cpp
    src/skills/GoToPos.cpp
    src/Aggregator.cpp
    src/LastWorld.cpp
)
add_dependencies(FollowPathTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(FollowPathTest ${catkin_LIBRARIES})

add_executable(AvoidRobotsTest
    src/skills/tests/AvoidRobotsTest.cpp
    src/skills/AvoidRobots.cpp
    src/skills/GoToPos.cpp
    src/Aggregator.cpp
    src/LastWorld.cpp
)
add_dependencies(AvoidRobotsTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(AvoidRobotsTest ${catkin_LIBRARIES})


catkin_add_gtest(tactics_test
    test/main.cpp 
	test/SimpleTest.cpp
)
target_link_libraries(tactics_test ${catkin_LIBRARIES})

add_executable(RoleNode
    src/RoleNode.cpp
    )
add_dependencies(RoleNode
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    generate_json_t
    )
target_link_libraries(RoleNode ${catkin_LIBRARIES})

add_executable(StrategyNode
    src/StrategyNode.cpp
    src/trees/alltrees.cpp
    src/LastWorld.cpp
    src/PickedTactic.cpp
    ${TACTICS_CONDITIONS_SOURCES}
    ${TACTICS_SKILLS_SOURCES}
    ${TACTICS_TACTICS_SOURCES}
    )
add_dependencies(StrategyNode
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    generate_json_t
    )
target_link_libraries(StrategyNode ${catkin_LIBRARIES})
