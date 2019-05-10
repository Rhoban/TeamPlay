#pragma once

#include <rhoban_utils/serialization/json_serializable.h>
#include <rhoban_team_play/extra_team_play.pb.h>
#include <hl_communication/wrapper.pb.h>
#include <cstdint>

namespace rhoban_team_play
{
// Broadcaster ports [UDP port]
#define TEAM_PLAY_PORT 29645
#define CAPTAIN_PORT 29646

// Broadcaster frequencies [Hz]
#define TEAM_PLAY_FREQUENCY 3
#define CAPTAIN_FREQUENCY 5

/// Maximal number of obstacles shared by each robot
#define MAX_OBSTACLES 10

/// Maximal number of opponents considered in consensus
#define MAX_OPPONENTS 5

/**
 * Robot playing state in teamplay
 */
enum TeamPlayState : int
{
  Inactive = 0,
  Playing = 1,
  BallHandling = 2,
  GoalKeeping = 8,
  Unknown = 16
};

/**
 * TeamPlay mMessage structure
 */
struct TeamPlayInfo
{
  // ID of the player
  int id;
  // State of the player
  TeamPlayState state;
  // Ball position in self frame
  float ballX, ballY, ballQ;
  bool ballOk;
  /// Ball speed along x-axis in self referential
  float ballVelX;
  /// Ball speed along y-axis in self referential
  float ballVelY;
  // Robot pose in field (fieldYaw is [rad])
  float fieldX, fieldY, fieldYaw, fieldQ, fieldConsistency;
  bool fieldOk;
  // Placing target
  bool placing;
  float targetX, targetY;
  float localTargetX, localTargetY;
  // Ball target
  float ballTargetX, ballTargetY;
  /// Time elapsed since last kick was performed
  float timeSinceLastKick;
  // Referee textual state
  char stateReferee[15];
  // Robocup textual state
  char stateRobocup[10];
  // Playing textual state
  char statePlaying[10];
  // Approach textual state
  char stateSearch[10];
  // Hardware warnings
  char hardwareWarnings[30];
  // Robot clock
  uint8_t hour, min, sec;
  /// Obstacles
  int nbObstacles;
  float obstaclesRadius;
  float obstacles[MAX_OBSTACLES][2];

  // Timestamp of data reception
  // in milliseconds
  float timestamp;

  // Is the player goal keeper ?
  bool goalKeeper;

  /**
   * Return the time in milliseconds
   * since data reception
   */
  float getAge() const;

  /**
   * Return true if data have been
   * received for too long
   */
  bool isOutdated() const;

  bool isPenalized() const;

  /**
   * Return the distance and the
   * azimuth between the robot
   * and the ball
   */
  float getBallDistance() const;
  float getBallAzimuth() const;

  /// Return the position of the ball in the field according to internal content
  /// Warning: this value is meaningless if !(ballOk && fieldOk)
  Eigen::Vector2d getBallInField() const;
};

void teamPlayfromJson(TeamPlayInfo& info, const Json::Value& json_value);
Json::Value teamPlayToJson(const TeamPlayInfo& info);

#define CAPTAIN_MAX_ID 6

enum CaptainOrder : int
{
  SearchBall = 0,
  HandleBall = 1,
  Place = 2
};

struct CommonBall
{
  /// Number of robots agreeing on the Ball position
  int nbRobots;
  /// Pos on field [m]
  float x;
  /// Pos on field [m]
  float y;
};

struct CommonOpponent
{
  /// Number of robots agreeing
  int consensusStrength;
  /// Pos on field [m]
  float x;
  /// Pos on field [m]
  float y;
};

struct CaptainInfo
{
  CaptainInfo();

  // Captain id
  int id;
  // Targets position & orientation for robots
  float robotTarget[CAPTAIN_MAX_ID][3];
  // Captain instruction for each robot
  CaptainOrder order[CAPTAIN_MAX_ID];
  // Reception timestamp
  float timestamp;

  /// Number of opponents in consensus
  int nb_opponents;
  /// List of the catkin opponents
  CommonOpponent common_opponents[MAX_OPPONENTS];

  CommonBall common_ball;

  /**
   * Return the time in milliseconds
   * since data reception
   */
  float getAge() const;

  /// Return the id of the robot handling the ball, if none of the robot is
  /// handling the ball, return -1
  /// Warning: ids start at '1'
  int getHandler() const;
};

void captainFromJson(CaptainInfo& info, const Json::Value& json_value);
Json::Value captainToJson(const CaptainInfo& info);

void exportTeamPlayToGameWrapper(const TeamPlayInfo& myInfo, int team_id, bool invert_field,
                                 hl_communication::GameMsg* dst);

void exportCaptain(const CaptainInfo& info, bool invert_field, hl_communication::Captain* captain);

PerceptionExtra extractPerceptionExtra(const hl_communication::Perception& msg);
MiscExtra extractMiscExtra(const hl_communication::RobotMsg& msg);

void invertField(hl_communication::RobotMsg* robot_msg);

/**
 * Return true if data are now considered as obsolete (older than 3 seconds)
 */
bool isOutdated(const hl_communication::RobotMsg& robot_msg);

}  // namespace rhoban_team_play
