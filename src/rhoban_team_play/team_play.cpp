#include <iostream>
#include <rhoban_team_play/team_play.h>
#include <rhoban_team_play/extra_team_play.pb.h>

#include <rhoban_utils/timing/time_stamp.h>
#include <hl_communication/utils.h>

#include <cmath>

using namespace rhoban_utils;
using namespace hl_communication;

namespace rhoban_team_play
{
void invertField(hl_communication::Perception* perception)
{
  for (int idx = 0; idx < perception->self_in_field_size(); idx++)
  {
    invertPose(perception->mutable_self_in_field(idx)->mutable_pose());
  }
}

void invertField(hl_communication::Intention* intention)
{
  if (intention->has_target_pose_in_field())
    invertPose(intention->mutable_target_pose_in_field());
  for (int idx = 0; idx < intention->waypoints_in_field_size(); idx++)
  {
    invertPose(intention->mutable_waypoints_in_field(idx));
  }
  if (intention->has_kick_target_in_field())
    invertPosition(intention->mutable_kick_target_in_field());
  if (intention->has_kick())
  {
    invertPosition(intention->mutable_kick()->mutable_start());
    invertPosition(intention->mutable_kick()->mutable_target());
  }
}

void invertField(hl_communication::Captain* captain)
{
  if (captain->has_ball())
  {
    invertPosition(captain->mutable_ball()->mutable_position());
  }
  for (int i = 0; i < captain->opponents_size(); i++)
  {
    invertPose(captain->mutable_opponents(i)->mutable_pose());
  }
  for (int i = 0; i < captain->orders_size(); i++)
  {
    StrategyOrder* order = captain->mutable_orders(i);
    if (order->has_target_pose())
    {
      invertPose(order->mutable_target_pose());
    }
  }
}

PerceptionExtra extractPerceptionExtra(const hl_communication::Perception& msg)
{
  PerceptionExtra result;
  if (msg.has_free_field())
  {
    result.ParseFromString(msg.free_field());
  }
  return result;
}

MiscExtra extractMiscExtra(const hl_communication::RobotMsg& msg)
{
  MiscExtra result;
  if (msg.has_free_field())
  {
    result.ParseFromString(msg.free_field());
  }
  return result;
}

void invertField(hl_communication::RobotMsg* robot_msg)
{
  if (robot_msg->has_perception())
    invertField(robot_msg->mutable_perception());
  if (robot_msg->has_intention())
    invertField(robot_msg->mutable_intention());
  if (robot_msg->has_captain())
    invertField(robot_msg->mutable_captain());
}

bool isOutdated(const hl_communication::RobotMsg& robot_msg)
{
  return (getTimeStamp() - robot_msg.time_stamp()) > 3 * 1000 * 1000;
}

}  // namespace rhoban_team_play
