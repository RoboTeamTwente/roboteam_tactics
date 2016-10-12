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

set(CMAKE_CXX_FLAGS "--std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

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

MESSAGE(${TACTICS_CONDITIONS_SOURCES})
MESSAGE(${TACTICS_SKILLS_SOURCES})
MESSAGE(${TACTICS_TACTICS_SOURCES})

add_executable(brain
	src/main.cpp
    src/Aggregator.cpp
    src/LastWorld.cpp
    src/PickedTactic.cpp
    src/trees/alltrees.cpp
    ${TACTICS_CONDITIONS_SOURCES}
    ${TACTICS_SKILLS_SOURCES}
    ${TACTICS_TACTICS_SOURCES}
)
add_dependencies(brain ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(brain ${catkin_LIBRARIES})

add_executable(TestSkill
	src/TestSkill.cpp
    src/Aggregator.cpp
    src/LastWorld.cpp
)
add_dependencies(TestSkill ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(TestSkill ${catkin_LIBRARIES})

add_executable(AimSkill
	src/AimSkill.cpp
    src/Aggregator.cpp
    src/LastWorld.cpp
)
add_dependencies(AimSkill ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(AimSkill ${catkin_LIBRARIES})

add_executable(KickSkill
	src/KickSkill.cpp
    src/Aggregator.cpp
    src/LastWorld.cpp
)
add_dependencies(KickSkill ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(KickSkill ${catkin_LIBRARIES})


catkin_add_gtest(tactics_test
    test/main.cpp 
	test/SimpleTest.cpp
)
target_link_libraries(tactics_test ${catkin_LIBRARIES})
