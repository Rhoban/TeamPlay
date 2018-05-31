#pragma once

#include <cstdint>

namespace rhoban_team_play
{

#define MAX_OBSTACLES 10

/**
 * Robot playing state in teamplay
 */
enum TeamPlayState : int {
    Inactive = 0,
    Playing = 1,
    BallHandling = 2,
    PlacingA = 3,
    PlacingB = 4,
    PlacingC = 5,
    PlacingD = 6,
    Unknown = 16
};

/**
 * TeamPlay mMessage structure
 */
struct TeamPlayInfo {
    //ID of the player
    int id;
    //State of the player
    TeamPlayState state;
    //Ball position in self frame
    float ballX, ballY, ballQ;
    bool ballOk;
    /// Ball speed along x-axis in self referential
    float ballVelX;
    /// Ball speed along y-axis in self referential
    float ballVelY;
    //Robot pose in field (fieldYaw is [rad])
    float fieldX, fieldY, fieldYaw, fieldQ, fieldConsistency;
    bool fieldOk;
    //Distance to placing
    float scoreA, scoreB, scoreC, scoreD;
    // Placing target
    bool placing;
    float targetX, targetY;
    float localTargetX, localTargetY;
    // Ball target
    float ballTargetX, ballTargetY;
    /// Time elapsed since last kick was performed
    float timeSinceLastKick;
    //Referee textual state
    char stateReferee[15];
    //Robocup textual state
    char stateRobocup[10];
    //Playing textual state
    char statePlaying[10];
    //Approach textual state
    char stateSearch[10];
    //Hardware warnings
    char hardwareWarnings[30];
    //Robot clock
    uint8_t hour, min, sec;
    /// Obstacles
    int nbObstacles;
    float obstaclesX[MAX_OBSTACLES];
    float obstaclesY[MAX_OBSTACLES];

    //Timestamp of data reception
    //in milliseconds
    float timestamp;

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

    /**
     * Score for a given role
     */
    float scoreFor(TeamPlayState role) const;

    /**
     * Return the distance and the
     * azimuth between the robot
     * and the ball
     */
    float getBallDistance() const;
    float getBallAzimuth() const;
};

#define CAPTAIN_MAX_ID  4

struct CaptainInfo
{
    // Captain id
    int id;
    // Targets position & orientation for robots
    float robotTarget[CAPTAIN_MAX_ID][3];
    // Reception timestamp
    float timestamp;
    
    /**
     * Return the time in milliseconds
     * since data reception
     */
    float getAge() const;
};

}