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

# Only enable this one if you're just testing one skill
set(BUILD_TESTX TRUE)

# Enable these if you're also testing strategies
set(BUILD_STRATEGYNODE TRUE)
set(BUILD_ROBOTNODE TRUE)

# Interesting debug feedback features
set(ENABLE_BT_TRACE TRUE)
set(ENABLE_DEBUG_GRAPHICS TRUE)

# Set defines based on settings
if (ENABLE_BT_TRACE)
    add_definitions(-DRTT_ENABLE_BT_RQT_TRACE)
endif (ENABLE_BT_TRACE)

if (ENABLE_DEBUG_GRAPHICS)
    add_definitions(-DRTT_ENABLE_DEBUG_GRAPHICS)
endif (ENABLE_DEBUG_GRAPHICS)

set(CMAKE_CXX_FLAGS "\
--std=c++14 ${CMAKE_CXX_FLAGS} \
-Wall \
-Wno-unused-function \
-Wsign-compare \
-g \
")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roboteam_tactics_movement
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_subdirectory(src)

# Executable that can convert behavior3 json to C++ functions
add_executable(converter
    src/treegen/BTBuilder.cpp
    src/treegen/converter.cpp
    )
add_dependencies(converter
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
target_link_libraries(converter
	${catkin_LIBRARIES}
	)

if (BUILD_TESTX)

    add_executable(TestX
        src/skills/tests/TestX.cpp
        $<TARGET_OBJECTS:alltrees>
        $<TARGET_OBJECTS:treegen>
        $<TARGET_OBJECTS:tactics>
        $<TARGET_OBJECTS:utils>
        $<TARGET_OBJECTS:skills>
        $<TARGET_OBJECTS:conditions>
        )
    add_dependencies(TestX 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
    target_link_libraries(TestX
        ${catkin_LIBRARIES}
        )

endif (BUILD_TESTX)

############################################################
#### Stupid stuff to get the json files to be converted ####
############################################################

add_custom_command(
    OUTPUT
        ${PROJECT_SOURCE_DIR}/generated/generate_cpp_from_json.stamp
    COMMAND ${PROJECT_SOURCE_DIR}/scripts/generate_cpp_from_json.sh
    DEPENDS
        ${JSON_FILES}
        converter
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Generating C++ from JSON files"
    )
add_custom_target(generate_json_t
    ALL
    DEPENDS
        ${PROJECT_SOURCE_DIR}/generated/generate_cpp_from_json.stamp
    )

#############################
#### End of stupid stuff ####
#############################

# Hooks that delete all generated files
file(GLOB GENERATED_FILES
    "include/roboteam_tactics/generated/*.h"
    )
set(GENERATED_FILES
    ${GENERATED_FILES}
    ${PROJECT_SOURCE_DIR}/generated/generate_debug_directives.stamp
    ${PROJECT_SOURCE_DIR}/generated/generate_cpp_from_json.stamp
    ${PROJECT_SOURCE_DIR}/generated/generate_all_leafs.stamp
    )


set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES ${GENERATED_FILES})
# End of hooks

catkin_add_gtest(tactics_test
    test/main.cpp
	test/SimpleTest.cpp
)
target_link_libraries(tactics_test ${catkin_LIBRARIES})

if (BUILD_ROBOTNODE)

    add_executable(RoleNode
        src/RoleNode.cpp
        $<TARGET_OBJECTS:treegen>
        $<TARGET_OBJECTS:alltrees>
        $<TARGET_OBJECTS:skills>
        $<TARGET_OBJECTS:conditions>
        $<TARGET_OBJECTS:tactics>
        $<TARGET_OBJECTS:utils>
        )
    add_dependencies(RoleNode
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        generate_json_t
        )
    target_link_libraries(RoleNode
        ${catkin_LIBRARIES}
        )

endif (BUILD_ROBOTNODE)

if (BUILD_STRATEGYNODE)

    add_executable(StrategyNode
        src/StrategyNode.cpp
        $<TARGET_OBJECTS:treegen>
        $<TARGET_OBJECTS:alltrees>
        $<TARGET_OBJECTS:skills>
        $<TARGET_OBJECTS:conditions>
        $<TARGET_OBJECTS:tactics>
        $<TARGET_OBJECTS:utils>
        )
    add_dependencies(StrategyNode
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        generate_json_t
        )
    target_link_libraries(StrategyNode
        ${catkin_LIBRARIES}
        )

endif (BUILD_STRATEGYNODE)

add_library(roboteam_tactics_movement
	src/skills/GoToPos.cpp
	src/skills/AvoidRobots.cpp

	src/utils/Cone.cpp
   	src/utils/DangerFinder.cpp
    src/utils/Draw.cpp
    src/utils/FeedbackCollector.cpp
    src/utils/LastWorld.cpp
    src/utils/LastRef.cpp
    src/utils/Math.cpp
    src/utils/RobotDealer.cpp
    src/utils/utils.cpp
    src/utils/BTRunner.cpp
    src/utils/BtDebug.cpp
	)
add_dependencies(roboteam_tactics_movement
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(roboteam_tactics_movement
        ${catkin_LIBRARIES}
        )	

### Test executables ###
# Disables tests on bob's desktop
site_name(computername)

# if(NOT (computername STREQUAL "lapbobe" OR computername STREQUAL "W-LENOVO-UBUNTU" OR computername STREQUAL "jim-SATELLITE-L50-B-2E4"))

    # add_executable(brain
        # src/main.cpp
    # )
    # add_dependencies(brain ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    # target_link_libraries(brain
        # ${catkin_LIBRARIES}
        # )

    # add_executable(KickAtGoal
        # src/role_tests/KickAtGoal.cpp
    # )
    # add_dependencies(KickAtGoal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} generate_json_t)
    # target_link_libraries(KickAtGoal
        # ${catkin_LIBRARIES}
        # )

    # add_executable(Keeper
        # src/role_tests/Keeper.cpp
    # )
    # add_dependencies(Keeper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} generate_json_t)
    # target_link_libraries(Keeper
        # ${catkin_LIBRARIES}
        # )
        
    # add_executable(RotateAroundPoint
        # src/skills/tests/RotateAroundPointTest.cpp
    # )
    # add_dependencies(RotateAroundPoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    # target_link_libraries(RotateAroundPoint
        # ${catkin_LIBRARIES}
        # )

    # add_executable(GoToPosTest
        # src/skills/tests/GoToPosTest.cpp
    # )
    # add_dependencies(GoToPosTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    # target_link_libraries(GoToPosTest
        # ${catkin_LIBRARIES}
        # )

    # add_executable(GetBallTest
        # src/skills/tests/GetBallTest.cpp
    # )
    # add_dependencies(GetBallTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    # target_link_libraries(GetBallTest
        # ${catkin_LIBRARIES}
        # )

	# add_executable(DribbleTest
        # src/skills/tests/DribbleTest.cpp
    # )
    # add_dependencies(DribbleTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    # target_link_libraries(DribbleTest
        # ${catkin_LIBRARIES}
        # )


    # add_executable(KickTest
        # src/skills/tests/KickTest.cpp
    # )
    # add_dependencies(KickTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    # target_link_libraries(KickTest
        # ${catkin_LIBRARIES}
        # )

    # add_executable(IcptTest
        # src/conditions/CanInterceptBall.cpp
    # )
    # add_dependencies(IcptTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    # target_link_libraries(IcptTest
        # ${catkin_LIBRARIES}
        # )
    # target_compile_options(IcptTest PRIVATE "-DDEBUG_ICPT")

    # catkin_add_gtest(VerificationTest
        # test/main.cpp
        # test/VerificationTest.cpp
    # )
    # target_link_libraries(VerificationTest ${catkin_LIBRARIES})

    # catkin_add_gtest(ConditionsTest
        # test/main.cpp
        # test/ConditionsTest.cpp
    # )
    # target_link_libraries(ConditionsTest
        # ${catkin_LIBRARIES}
        # )

    # add_dependencies(KickTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    # target_link_libraries(KickTest ${catkin_LIBRARIES})

    # add_executable(FollowPathTest
        # src/skills/tests/FollowPathTest.cpp
    # )
    # add_dependencies(FollowPathTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    # target_link_libraries(FollowPathTest
        # ${catkin_LIBRARIES}
        # )

    # add_executable(AvoidRobotsTest
        # src/skills/tests/AvoidRobotsTest.cpp
    # )
    # add_dependencies(AvoidRobotsTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    # target_link_libraries(AvoidRobotsTest
        # ${catkin_LIBRARIES}
        # )

    # add_executable(CanPassSafelyTest
        # src/conditions/tests/CanPassSafelyTest.cpp
    # )
    # add_dependencies(CanPassSafelyTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    # target_link_libraries(CanPassSafelyTest
        # ${catkin_LIBRARIES}
        # )

    # add_executable(CanReachPointTest
        # src/conditions/tests/CanReachPointTest.cpp
    # )
    # add_dependencies(CanReachPointTest
        # ${${PROJECT_NAME}_EXPORTED_TARGETS}
        # ${catkin_EXPORTED_TARGETS}
        # )
    # target_link_libraries(CanReachPointTest
        # ${catkin_LIBRARIES}
        # )

# add_executable(ConeTest
        # src/utils/tests/ConeTest.cpp
    # )
    # add_dependencies(ConeTest
        # ${${PROJECT_NAME}_EXPORTED_TARGETS}
        # ${catkin_EXPORTED_TARGETS}
        # )
    # target_link_libraries(ConeTest
        # ${catkin_LIBRARIES}
        # )

# add_executable(CanSeeRobotTest
        # src/conditions/tests/CanSeeRobotTest.cpp
    # )
    # add_dependencies(CanSeeRobotTest
        # ${${PROJECT_NAME}_EXPORTED_TARGETS}
        # ${catkin_EXPORTED_TARGETS}
        # )
    # target_link_libraries(CanSeeRobotTest
        # ${catkin_LIBRARIES}
        # )

# add_executable(AttackTactic
        # src/role_tests/AttackTactic.cpp
    # )
    # add_dependencies(AttackTactic
        # ${${PROJECT_NAME}_EXPORTED_TARGETS}
        # ${catkin_EXPORTED_TARGETS}
        # )
    # target_link_libraries(AttackTactic
        # ${catkin_LIBRARIES}
        # )

# add_executable(StandFreeTest
        # src/skills/tests/StandFreeTest.cpp
    # )
    # add_dependencies(StandFreeTest
        # ${${PROJECT_NAME}_EXPORTED_TARGETS}
        # ${catkin_EXPORTED_TARGETS}
        # )
    # target_link_libraries(StandFreeTest
        # ${catkin_LIBRARIES}
        # )

# endif(NOT (computername STREQUAL "lapbobe" OR computername STREQUAL "W-LENOVO-UBUNTU" OR computername STREQUAL "jim-SATELLITE-L50-B-2E4"))
