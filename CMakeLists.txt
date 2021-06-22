cmake_minimum_required(VERSION 3.16.3)
project(rhoban_team_play)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -Wno-unused-parameter")

find_package(Protobuf REQUIRED)

set(CMAKE_CURRENT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/rhoban_team_play)

protobuf_generate_cpp(PROTO_SOURCES PROTO_HEADERS
  proto/extra_team_play.proto
)

add_library (rhoban_team_play
  src/rhoban_team_play/team_play.cpp
  ${PROTO_SOURCES}
  ${PROTO_HEADERS}
)
target_link_libraries(rhoban_team_play PUBLIC
  rhoban_utils
  hl_communication
  protobuf::libprotobuf
)
target_include_directories(rhoban_team_play SYSTEM PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}/..
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
