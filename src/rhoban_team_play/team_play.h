#pragma once

#include <rhoban_utils/serialization/json_serializable.h>
#include <rhoban_team_play/extra_team_play.pb.h>
#include <hl_communication/wrapper.pb.h>
#include <cstdint>

namespace rhoban_team_play
{
PerceptionExtra extractPerceptionExtra(const hl_communication::Perception& msg);
MiscExtra extractMiscExtra(const hl_communication::RobotMsg& msg);

void invertField(hl_communication::RobotMsg* robot_msg);

/**
 * Return true if data are now considered as obsolete (older than 3 seconds)
 */
bool isOutdated(const hl_communication::RobotMsg& robot_msg);

}  // namespace rhoban_team_play
