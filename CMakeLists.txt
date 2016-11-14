cmake_minimum_required(VERSION 2.8.3)
project(roboteam_tactics)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(catkin REQUIRED COMPONENTS
  roboteam_msgs
  roboteam_utils
  roscpp
  std_msgs
  actionlib_msgs
  actionlib
  uuid_msgs
  unique_id
)

set(CMAKE_CXX_FLAGS "\
--std=c++14 ${CMAKE_CXX_FLAGS} \
-Wall \
-Wno-unused-function \
-Wsign-compare \
-fmax-errors=1 \
-g \
")

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_subdirectory(src)


add_executable(DoStrategy
    src/role_tests/DoStrategy.cpp
)
add_dependencies(DoStrategy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} generate_json_t)
target_link_libraries(DoStrategy
	alltrees
    skills
    conditions
    tactics
    utils 
	${catkin_LIBRARIES}
	)

# Executable that can convert behavior3 json to C++ functions
add_executable(converter
    src/treegen/BTBuilder.cpp
    src/treegen/converter.cpp
    )
add_dependencies(converter generate_leafs_t)

add_executable(TestX
    src/skills/tests/TestX.cpp
    )
add_dependencies(TestX ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} generate_leafs_t)
target_link_libraries(TestX
    tactics
    utils
    skills
    conditions
    ${catkin_LIBRARIES}
    )

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
target_link_libraries(RoleNode 
    alltrees
    skills
    conditions
    tactics
    utils
    ${catkin_LIBRARIES}
    )

add_executable(StrategyNode
    src/StrategyNode.cpp
    )
add_dependencies(StrategyNode
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    generate_json_t
    )
target_link_libraries(StrategyNode
    alltrees
    skills
    conditions
    tactics
    utils
    ${catkin_LIBRARIES}
    )

### Test executables ###
# Disables tests on bob's desktop
site_name(computername)

if(NOT computername STREQUAL "minux")

    add_executable(brain
        src/main.cpp
    )
    add_dependencies(brain ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(brain
        alltrees
        skills
        conditions
        tactics
        utils
        ${catkin_LIBRARIES}
        )

    add_executable(KickAtGoal
        src/role_tests/KickAtGoal.cpp
    )
    add_dependencies(KickAtGoal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} generate_json_t)
    target_link_libraries(KickAtGoal
        alltrees
        skills
        conditions
        tactics
        utils
        ${catkin_LIBRARIES}
        )

    add_executable(Keeper
        src/role_tests/Keeper.cpp
    )
    add_dependencies(Keeper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} generate_json_t)
    target_link_libraries(Keeper
        alltrees
        skills
        conditions
        tactics
        utils
        ${catkin_LIBRARIES}
        )

    add_executable(AimSkill
        src/AimSkill.cpp
    )
    add_dependencies(AimSkill 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
    target_link_libraries(AimSkill
        utils
        skills
        conditions
        ${catkin_LIBRARIES}
        )


    add_executable(RotateAroundPoint
        src/skills/tests/RotateAroundPointTest.cpp
    )
    add_dependencies(RotateAroundPoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(RotateAroundPoint 
        skills
        conditions
        utils
        ${catkin_LIBRARIES}
        )

    add_executable(GoToPosTest
        src/skills/tests/GoToPosTest.cpp
    )
    add_dependencies(GoToPosTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(GoToPosTest
        skills
        conditions
        utils
        ${catkin_LIBRARIES}
        )

    add_executable(GetBallTest
        src/skills/tests/GetBallTest.cpp
    )
    add_dependencies(GetBallTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(GetBallTest 
        skills
        conditions
        utils
        ${catkin_LIBRARIES}
        )

    add_executable(KickTest
        src/skills/tests/KickTest.cpp
    )
    add_dependencies(KickTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(KickTest
        skills
        conditions
        utils
        ${catkin_LIBRARIES}
        )

    add_executable(IcptTest
        src/conditions/CanInterceptBall.cpp
    )
    add_dependencies(IcptTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(IcptTest 
        utils
        conditions
        skills
        ${catkin_LIBRARIES}
        )
    target_compile_options(IcptTest PRIVATE "-DDEBUG_ICPT")

    catkin_add_gtest(VerificationTest
        test/main.cpp
        test/VerificationTest.cpp
    )
    target_link_libraries(VerificationTest ${catkin_LIBRARIES})

    catkin_add_gtest(ConditionsTest
        test/main.cpp
        test/ConditionsTest.cpp
    )
    target_link_libraries(ConditionsTest 
        conditions
        utils
        ${catkin_LIBRARIES}
        )

    add_dependencies(KickTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(KickTest ${catkin_LIBRARIES})

    add_executable(FollowPathTest
        src/skills/tests/FollowPathTest.cpp
    )
    add_dependencies(FollowPathTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(FollowPathTest
        skills
        conditions
        utils
        ${catkin_LIBRARIES}
        )

    add_executable(AvoidRobotsTest
        src/skills/tests/AvoidRobotsTest.cpp
    )
    add_dependencies(AvoidRobotsTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(AvoidRobotsTest 
        skills
        conditions
        utils
        ${catkin_LIBRARIES}
        )

    add_executable(CanPassSafelyTest
        src/conditions/tests/CanPassSafelyTest.cpp
    )
    add_dependencies(CanPassSafelyTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(CanPassSafelyTest 
        skills
        conditions
        utils
        ${catkin_LIBRARIES}
        )

    add_executable(CanReachPointTest
        src/conditions/tests/CanReachPointTest.cpp
    )
    add_dependencies(CanReachPointTest
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
    target_link_libraries(CanReachPointTest 
        skills
        conditions
        utils
        ${catkin_LIBRARIES}
        )

add_executable(ConeTest
        src/utils/tests/ConeTest.cpp
    )
    add_dependencies(ConeTest
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
    target_link_libraries(ConeTest 
        skills
        conditions
        utils
        ${catkin_LIBRARIES}
        )

add_executable(StandFreeTest
        src/skills/tests/StandFreeTest.cpp
    )
    add_dependencies(StandFreeTest
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
    target_link_libraries(StandFreeTest 
        skills
        conditions
        utils
        ${catkin_LIBRARIES}
        )

add_executable(StandFreeAndReceive
        src/role_tests/StandFreeAndReceive.cpp
    )
    add_dependencies(StandFreeAndReceive
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
    target_link_libraries(StandFreeAndReceive 
        skills
        conditions
        utils
        ${catkin_LIBRARIES}
        )

endif(NOT computername STREQUAL "minux")
