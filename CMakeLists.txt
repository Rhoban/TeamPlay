cmake_minimum_required(VERSION 2.8)
project(rhoban_team_play)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -Wno-unused-parameter")

include(FindProtobuf)
find_package(Protobuf REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rhoban_utils
  hl_communication
  )

set (PROTOBUF_MESSAGES
  proto/extra_team_play.proto
  )

# Handling generated headers is based on Dirk Thomas answer on topic:
# - https://answers.ros.org/answers/123416/revisions/
catkin_destinations()
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION})
protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS ${PROTOBUF_MESSAGES})

set_source_files_properties(${PROTO_SOURCES} ${PROTO_HEADERS} PROPERTIES GENERATED TRUE)

set(PROTO_DUMMY_FILE proto_generation)

add_custom_command(
  OUTPUT ${PROTO_DUMMY_FILE}
  COMMAND cp ${PROTO_HEADERS} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  COMMAND touch ${PROTO_DUMMY_FILE}
  DEPENDS ${PROTO_HEADERS}
  )

set(DELEGATE_INCLUDE_DIRS
  include
  ${PROTOBUF_INCLUDE_DIR}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})


catkin_package(
    INCLUDE_DIRS ${DELEGATE_INCLUDE_DIRS}
    LIBRARIES rhoban_team_play ${PROTOBUF_LIBRARY}
    CATKIN_DEPENDS rhoban_utils hl_communication
    )

include_directories(${DELEGATE_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library (rhoban_team_play src/rhoban_team_play/team_play.cpp ${PROTO_SOURCES} ${PROTO_DUMMY_FILE})
target_link_libraries(rhoban_team_play ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})
