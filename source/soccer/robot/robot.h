#pragma once

#include "velocity_profile.h"

#define PREDICT_CMDS 6

namespace Tyr::Soccer
{
class Robot
{
public:
    Common::RobotState target;
    int                shoot, chip, dribbler;
    unsigned char      data[32];
    int                vision_id;
    bool               halted;

    Sender::Command lastCMDs[11];
    int             last_cmd_idx = 0;

    int CMDindex;

    Robot();

    inline const Common::RobotState &state() const
    {
        return Common::worldState().own_robot[vision_id];
    }

    void setVisionId(unsigned short v_id);

    void Shoot(int pow);

    void Chip(int pow);

    void Dribble(int pow);

    void face(Common::Vec2 _target);

    void MoveByMotion(Common::Vec3 motion);

    Common::Vec3 ComputeMotionCommand(float speed, const VelocityProfile &velocityProfile);

    Sender::Command GetCurrentCommand() const;

    void makeSendingDataReady();
};
} // namespace Tyr::Soccer
