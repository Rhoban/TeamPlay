#include <iostream>
#include "rhoban_team_play/team_play.h"

#include <rhoban_utils/timing/time_stamp.h>

#include <cmath>

using namespace rhoban_utils;

namespace rhoban_team_play
{

void teamPlayfromJson(TeamPlayInfo &info, const Json::Value & json_value)
{
    if (json_value.size() == 32) {
        int k = 0;
        
        // Robot id and state
        info.id = json_value[k++].asInt();
        info.state = (TeamPlayState)json_value[k++].asInt();
        
        // Ball
        info.ballX = json_value[k++].asFloat();
        info.ballY = json_value[k++].asFloat();
        info.ballQ = json_value[k++].asFloat();
        info.ballOk = json_value[k++].asBool();
        info.ballVelX = json_value[k++].asFloat();
        info.ballVelY = json_value[k++].asFloat();
        
        // Field
        info.fieldX = json_value[k++].asFloat();
        info.fieldY = json_value[k++].asFloat();
        info.fieldYaw = json_value[k++].asFloat();
        info.fieldQ = json_value[k++].asFloat();
        info.fieldConsistency = json_value[k++].asFloat();
        info.fieldOk = json_value[k++].asBool();
        
        // Placing data
        info.placing = json_value[k++].asBool();
        info.targetX = json_value[k++].asFloat();
        info.targetY = json_value[k++].asFloat();
        info.localTargetX = json_value[k++].asFloat();
        info.localTargetY = json_value[k++].asFloat();
        
        // Kick
        info.ballTargetX = json_value[k++].asFloat();
        info.ballTargetY = json_value[k++].asFloat();
        info.timeSinceLastKick = json_value[k++].asFloat();
        
        // States
        strncpy(info.stateReferee, json_value[k++].asString().c_str(), sizeof(info.stateReferee));
        strncpy(info.stateRobocup, json_value[k++].asString().c_str(), sizeof(info.stateRobocup));
        strncpy(info.statePlaying, json_value[k++].asString().c_str(), sizeof(info.statePlaying));
        strncpy(info.stateSearch, json_value[k++].asString().c_str(), sizeof(info.stateSearch));
        strncpy(info.hardwareWarnings, json_value[k++].asString().c_str(), sizeof(info.hardwareWarnings));
        
        // Hour
        info.hour = json_value[k++].asInt();
        info.min = json_value[k++].asInt();
        info.sec = json_value[k++].asInt();
        
        // Obstacles
        info.obstaclesRadius = json_value[k++].asFloat();
        info.nbObstacles = 0;
        for (auto obstacle : json_value[k++]) {
            info.obstacles[info.nbObstacles][0] = obstacle[0].asFloat();
            info.obstacles[info.nbObstacles][1] = obstacle[1].asFloat();
            info.nbObstacles++;
        }
        
        info.timestamp = json_value[k++].asFloat();
    } else {
        std::cerr << "TeamPlayInfo::fromJson bad array size!" << std::endl;    
    }
}
                    
Json::Value teamPlayToJson(const TeamPlayInfo &info)
{
    Json::Value json(Json::arrayValue);
    
    // Robot id and state
    json.append(info.id);
    json.append(info.state);
    
    // Ball
    json.append(info.ballX);
    json.append(info.ballY);
    json.append(info.ballQ);
    json.append(info.ballOk);
    json.append(info.ballVelX);
    json.append(info.ballVelY);
    
    // Field
    json.append(info.fieldX);
    json.append(info.fieldY);
    json.append(info.fieldYaw);
    json.append(info.fieldQ);
    json.append(info.fieldConsistency);
    json.append(info.fieldOk);
    
    // Placing data
    json.append(info.placing);
    json.append(info.targetX);
    json.append(info.targetY);
    json.append(info.localTargetX);
    json.append(info.localTargetY);
    
    // Kick info
    json.append(info.ballTargetX);
    json.append(info.ballTargetY);
    json.append(info.timeSinceLastKick);
    
    // States
    json.append(info.stateReferee);
    json.append(info.stateRobocup);
    json.append(info.statePlaying);
    json.append(info.stateSearch);
    json.append(info.hardwareWarnings);
    
    // Timing
    json.append(info.hour);
    json.append(info.min);
    json.append(info.sec);
    
    // Obstacles
    json.append(info.obstaclesRadius);
    Json::Value obstacles = Json::Value(Json::arrayValue);
    for (int k=0; k<info.nbObstacles; k++) {
        Json::Value obstacle(Json::arrayValue);
        obstacle[0] = info.obstacles[k][0];
        obstacle[1] = info.obstacles[k][1];
        obstacles.append(obstacle);
    }
    json.append(obstacles);
    
    // Timestamp
    json.append(info.timestamp);
    
    return json;
}

float TeamPlayInfo::getAge() const
{
    return TimeStamp::now().getTimeMS() - timestamp;
}

bool TeamPlayInfo::isOutdated() const
{
    //Outdated after 3 seconds
    return (getAge() > 3000);
}

float TeamPlayInfo::getBallDistance() const
{
    return sqrt(ballX*ballX + ballY*ballY);
}

float TeamPlayInfo::getBallAzimuth() const
{
    return atan2(ballY, ballX);
}

CaptainInfo::CaptainInfo()
: id(-1)
{    
}

float CaptainInfo::getAge() const
{
    return TimeStamp::now().getTimeMS() - timestamp;
}

void captainFromJson(CaptainInfo &info, const Json::Value & json_value)
{
    if (json_value.size() == 4) {
        int k = 0;
        
        info.id = json_value[k++].asInt();
        
        auto targets = json_value[k++];
        for (int n=0; n<CAPTAIN_MAX_ID; n++) {
            info.robotTarget[n][0] = targets[n][0].asFloat();
            info.robotTarget[n][1] = targets[n][1].asFloat();
            info.robotTarget[n][2] = targets[n][2].asFloat();
        }
        
        auto orders = json_value[k++];
        for (int n=0; n<CAPTAIN_MAX_ID; n++) {
            info.order[n] = (CaptainOrder)orders[n].asInt();
        }
        
        info.timestamp = json_value[k++].asFloat();
    } else {
        std::cerr << "CaptainInfo bad json size" << std::endl;    
    }
}
                    
Json::Value captainToJson(const CaptainInfo &info)
{
    Json::Value json;
    
    json.append(info.id);
    
    Json::Value targets(Json::arrayValue);
    for (int k=0; k<CAPTAIN_MAX_ID; k++) {
        Json::Value target(Json::arrayValue);
        target.append(info.robotTarget[k][0]);
        target.append(info.robotTarget[k][1]);
        target.append(info.robotTarget[k][2]);
        targets.append(target);
    }
    json.append(targets);
    
    Json::Value orders(Json::arrayValue);
    for (int k=0; k<CAPTAIN_MAX_ID; k++) {
        orders.append(info.order[k]);
    }
    json.append(orders);
    
    json.append(info.timestamp);
    
    return json;
}

}
