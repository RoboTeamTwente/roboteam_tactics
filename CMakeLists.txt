cmake_minimum_required(VERSION 2.8.3)
project(roboteam_tactics)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# Colors for everyone to use!
if(NOT WIN32)
    string(ASCII 27 Esc)
    set(BLUE_TEXT "${Esc}[34m")
    set(BOLD_TEXT  "${Esc}[1m")
    set(RESET_TEXT "${Esc}[0m")
endif()

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

# This part actually makes sure the network interface
# of Qt4 is included! So while rqt uses Qt5, for our
# networking purposes we use Qt4.
set(QT_USE_QTNETWORK TRUE)
find_package(Qt REQUIRED)
include(${QT_USE_FILE})

# Of what we use, only filesystem requires linking
# and is therefore mentoined as component. Mostly
# everything else is header only, and thus doesn't
# need to be mentioned here.
# Dennis and Bob had some linking problems with newer
# versions of boost (>= 1.63.0, some boost::filesystem
# code produced strange segfaults), so that's why we
# fixed it here to get a clear error when the version
# changes.
find_package(Boost
    1.58.0 EXACT
    REQUIRED
    COMPONENTS
        filesystem
    )

# Only enable this one if you're just testing one skill
set(BUILD_TESTX TRUE)

# Enable these if you're also testing strategies
set(BUILD_STRATEGYNODE TRUE)
set(BUILD_ROBOTNODE TRUE)
set(BUILD_PRACTICE_TESTS TRUE)

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
    INCLUDE_DIRS 
        include
    LIBRARIES
        roboteam_tactics_analysis
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
        $<TARGET_OBJECTS:analysis>
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

# 23/12/2016
# I left the stamp stuff commented because I might use it in the next
# week or so. I'm not sure if the current approach works consistently enough.

set(alltrees_output_file
    ${PROJECT_SOURCE_DIR}/src/trees/alltrees.cpp
    )

add_custom_command(
    OUTPUT
        # ${PROJECT_SOURCE_DIR}/generated/generate_cpp_from_json.stamp
        ${alltrees_output_file}
    COMMAND ${PROJECT_SOURCE_DIR}/scripts/generate_cpp_from_json.sh
    DEPENDS
        ${JSON_FILES}
        converter
        ${PROJECT_SOURCE_DIR}/scripts/generate_cpp_from_json.sh
    WORKING_DIRECTORY 
        ${PROJECT_SOURCE_DIR}
    COMMENT 
        "Generating C++ from JSON files"
    )
add_custom_target(generate_json_t
    ALL
    DEPENDS
        # ${PROJECT_SOURCE_DIR}/generated/generate_cpp_from_json.stamp
        ${alltrees_output_file}
    )

# Generated files from json trees
set(GENERATED_FILES
    ${GENERATED_FILES}
    ${PROJECT_SOURCE_DIR}/include/roboteam_tactics/generated/alltrees.h
    ${PROJECT_SOURCE_DIR}/src/trees/alltrees.cpp
    ${PROJECT_SOURCE_DIR}/generated/generate_cpp_from_json.stamp
    )

foreach(json_project ${JSON_PROJECTS})

    get_filename_component(project_basename ${json_project} NAME_WE)

    set(timestamp_file ${PROJECT_SOURCE_DIR}/generated/generate_cpp_from_json_project_${project_basename}.stamp)

    set(output_files
        ${PROJECT_SOURCE_DIR}/include/roboteam_tactics/generated/${project_basename}_trees.h
        ${PROJECT_SOURCE_DIR}/src/trees/${project_basename}_trees.cpp
        )

    add_custom_command(
        OUTPUT
            # ${timestamp_file}
            ${output_files}
        COMMAND
            ${PROJECT_SOURCE_DIR}/scripts/generate_cpp_from_json_project.sh ${json_project}
        DEPENDS
            ${json_project}
            converter
            ${PROJECT_SOURCE_DIR}/scripts/generate_cpp_from_json_project.sh
        WORKING_DIRECTORY
            ${PROJECT_SOURCE_DIR}
        COMMENT
            "Generating C++ from B3 project \"${project_basename}\""
        )

    add_custom_target(generate_json_project_${project_basename}_t
        ALL
        DEPENDS
            # ${timestamp_file}
            ${output_files}
        )

    MESSAGE("-- Project files for " ${BLUE_TEXT} ${BOLD_TEXT} ${project_basename} ${RESET_TEXT} ": ")
    
    foreach(output_file ${output_files})
        MESSAGE("--     - ${output_file}")
    endforeach(output_file)

    set(GENERATED_FILES
        ${GENERATED_FILES}
        # ${timestamp_file}
        # ${PROJECT_SOURCE_DIR}/include/roboteam_tactics/generated/${project_basename}_trees.h
        # ${PROJECT_SOURCE_DIR}/src/trees/${project_basename}_trees.cpp
        ${output_files}
        )

endforeach(json_project)

set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES ${GENERATED_FILES})

#############################
#### End of stupid stuff ####
#############################

catkin_add_gtest(tactics_test
    test/main.cpp
    test/SimpleTest.cpp
    test/ConditionsTest.cpp
    test/VerificationTest.cpp
    test/SpaceTest.cpp
    test/ParamsStressTest.cpp
    $<TARGET_OBJECTS:analysis>
    $<TARGET_OBJECTS:conditions>
    $<TARGET_OBJECTS:tactics>
    $<TARGET_OBJECTS:utils>
    $<TARGET_OBJECTS:skills>
)
target_link_libraries(tactics_test ${catkin_LIBRARIES})

if (BUILD_ROBOTNODE)

    add_executable(RoleNode
        src/RoleNode.cpp
        $<TARGET_OBJECTS:analysis>
        $<TARGET_OBJECTS:treegen>
        $<TARGET_OBJECTS:alltrees>
        $<TARGET_OBJECTS:skills>
        $<TARGET_OBJECTS:conditions>
        $<TARGET_OBJECTS:tactics>
        $<TARGET_OBJECTS:utils>
	$<TARGET_OBJECTS:rules>
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
        $<TARGET_OBJECTS:analysis>
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

catkin_add_gtest(BtTest
    test/main.cpp
    test/BtTest.cpp
)
target_link_libraries(BtTest ${catkin_LIBRARIES})

#   add_executable(Learner
#        src/utils/tests/Learner.cpp
#        $<TARGET_OBJECTS:treegen>
#        $<TARGET_OBJECTS:alltrees>
#        $<TARGET_OBJECTS:skills>
#        $<TARGET_OBJECTS:conditions>
#        $<TARGET_OBJECTS:tactics>
#        $<TARGET_OBJECTS:utils>
#        )
#    add_dependencies(Learner
#        ${${PROJECT_NAME}_EXPORTED_TARGETS}
#        ${catkin_EXPORTED_TARGETS}
#        
#        generate_json_t
#        )
#    target_link_libraries(Learner
#        ${catkin_LIBRARIES}
#	${QT_LIBRARIES}
#        )

if (BUILD_PRACTICE_TESTS)

    add_executable(PracticeTests
        ./src/practice_tests/main.cpp
        ./src/practice_tests/PracticeTest.cpp
        ./src/practice_tests/KeeperTest.cpp
        ./src/practice_tests/TestSetup.cpp
        ./src/practice_tests/FreeKickTest.cpp

        $<TARGET_OBJECTS:analysis>
        $<TARGET_OBJECTS:treegen>
        $<TARGET_OBJECTS:alltrees>
        $<TARGET_OBJECTS:skills>
        $<TARGET_OBJECTS:conditions>
        $<TARGET_OBJECTS:tactics>
        $<TARGET_OBJECTS:utils>
        )
    add_dependencies(PracticeTests
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        generate_json_t
	utils
        )
    target_link_libraries(PracticeTests
        ${catkin_LIBRARIES}
        # For networking
        ${QT_LIBRARIES}
        )

endif (BUILD_PRACTICE_TESTS)
