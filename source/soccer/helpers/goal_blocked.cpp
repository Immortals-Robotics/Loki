#include "../ai.h"

namespace Tyr::Soccer
{
bool Ai::goalBlocked(Common::Vec2 init_pos, float max_shoot_blocker_dis, float shoot_blocker_r)
{
    bool oppGoalOpen = true;

    Common::Line ballGoalLine = Common::Line::fromTwoPoints(oppGoal(), init_pos);
    for (int i = 0; i < Common::Setting::kMaxRobots; i++)
    {
        if (m_world_state.opp_robot[i].seen_state == Common::SeenState::CompletelyOut)
            continue;
        if ((std::fabs(m_world_state.opp_robot[i].position.x) > Common::field().width) ||
            (std::fabs(m_world_state.opp_robot[i].position.y) > Common::field().height))
            continue;
        if (m_world_state.opp_robot[i].position.distanceTo(init_pos) > max_shoot_blocker_dis)
            continue;

        if (ballGoalLine.distanceTo(m_world_state.opp_robot[i].position) < shoot_blocker_r)
        {
            oppGoalOpen = false;
            break;
        }
    }

    return !oppGoalOpen;
}
} // namespace Tyr::Soccer
