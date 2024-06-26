#include "../ai.h"

namespace Tyr::Soccer
{
void Ai::halt(const int t_robot_num)
{
    m_own_robot[t_robot_num].halt();
}

void Ai::haltAll()
{
    for (int i = 0; i < Common::Config::Common::kMaxRobots; i++)
    {
        halt(i);
    }
}
} // namespace Tyr::Soccer
