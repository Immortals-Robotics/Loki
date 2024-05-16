#include "../ai.h"

namespace Tyr::Soccer
{
void Ai::Mark2Goal(int robot_num, int opp, float dist)
{
    const float opp_predict_t = 0.15;
    dist                      = std::min(1500.0f, dist);

    Common::Vec2 predictedOpp =
        m_world_state.opp_robot[opp].position + m_world_state.opp_robot[opp].velocity * opp_predict_t;
    Common::Vec2 target = predictedOpp.pointOnConnectingLine(ownGoal(), dist);

    OwnRobot[robot_num].face(m_world_state.ball.position);
    setObstacles(robot_num, true);
    navigate(robot_num, target, VelocityProfile::mamooli());
}
} // namespace Tyr::Soccer
