#include "../ai.h"

namespace Tyr::Soccer
{
void Ai::DefHi(int robot_num, Common::Vec2 *defendTarget, bool stop)
{
    // side = -side;
    if (!defendTarget)
        defendTarget = &(m_world_state.ball.position);

    const float max_def_move_to_intercept    = 400.0f;
    const float max_near_asshole_dis         = 750.0f;
    const float max_ball_handler_asshole_dis = 400.0f;
    const float max_shoot_blocker_dis        = 1500.0f;
    const float shoot_blocker_r              = 200.0f;
    const float max_fast_ball_spd            = 5000.0f;
    const float max_own_attacker_dis         = 400.0f;

    bool ballAriving      = false;
    bool oneTouchNear     = false;
    bool interceptNear    = false;
    bool assholeNear      = false;
    bool assholeHasBall   = false;
    bool oppGoalOpen      = false;
    bool ballMovingFast   = false;
    bool ownAttackHasBall = false;
    bool ballIsToGoal     = false;

    Common::Vec2 target = DefGhuz(defendTarget);

    ballAriving = oneTouchDetector[robot_num].IsArriving(40, 80);

    Common::Vec2 oneTouchPos = CalculatePassPos(robot_num, Common::Vec2(-side * Common::field().width, 0),
                                                OwnRobot[robot_num].state().position);

    if (oneTouchPos.distanceTo(target) < max_def_move_to_intercept)
    {
        oneTouchNear = true;
    }

    if (m_world_state.ball.position.distanceTo(target) < max_def_move_to_intercept)
    {
        interceptNear = true;
    }

    int nearestAsshole = findNearestAsshole(target, -1, true);
    assholeNear = m_world_state.opp_robot[nearestAsshole].position.distanceTo(target) < max_near_asshole_dis;

    int ballHandlerAsshole = findNearestAsshole(m_world_state.ball.position, -1, true);
    assholeHasBall         = m_world_state.opp_robot[ballHandlerAsshole].position.distanceTo(
                         m_world_state.ball.position) < max_ball_handler_asshole_dis;

    oppGoalOpen               = true;
    Common::Line ballGoalLine = Common::Line::fromTwoPoints(Common::Vec2(-Common::field().width * side, 0),
                                                            Common::Vec2(target.x, target.y));
    for (int i = 0; i < Common::Setting::kMaxRobots; i++)
    {
        if (m_world_state.opp_robot[i].seen_state == Common::SeenState::CompletelyOut)
            continue;
        if ((std::fabs(m_world_state.opp_robot[i].position.x) > Common::field().width) ||
            (std::fabs(m_world_state.opp_robot[i].position.y) > Common::field().height))
            continue;
        if (m_world_state.opp_robot[i].position.distanceTo(target) > max_shoot_blocker_dis)
            continue;

        if (ballGoalLine.distanceTo(m_world_state.opp_robot[i].position) < shoot_blocker_r)
        {
            oppGoalOpen = false;
            break;
        }
    }
    if (oppGoalOpen)
    {
        for (int i = 0; i < Common::Setting::kMaxOnFieldTeamRobots; i++)
        {
            if (OwnRobot[i].state().seen_state == Common::SeenState::CompletelyOut)
                continue;
            if ((std::fabs(OwnRobot[i].state().position.x) > Common::field().width) ||
                (std::fabs(OwnRobot[i].state().position.y) > Common::field().height))
                continue;
            if (OwnRobot[i].state().position.distanceTo(target) > max_shoot_blocker_dis)
                continue;

            if (ballGoalLine.distanceTo(Common::Vec2(OwnRobot[i].state().position.x, OwnRobot[i].state().position.y)) <
                shoot_blocker_r)
            {
                oppGoalOpen = false;
                break;
            }
        }
    }

    ballMovingFast = m_world_state.ball.velocity.length() > max_fast_ball_spd;

    ownAttackHasBall =
        OwnRobot[attack].state().position.distanceTo(m_world_state.ball.position) < max_own_attacker_dis;
    if (OwnRobot[attack].state().seen_state == Common::SeenState::CompletelyOut)
    {
        ownAttackHasBall = false;
    }

    ballIsToGoal = ballIsGoaling();
    ballIsToGoal = false;

    Common::logDebug("interceptNear:{}", interceptNear);
    Common::logDebug("assholeHasBall: {}", assholeHasBall);
    Common::logDebug("ballMovingFast: {}", ballMovingFast);
    Common::logDebug("ownAttackHasBall: {}", ownAttackHasBall);
    Common::logDebug("ballIsToGoal: {}", ballIsToGoal);
    Common::logDebug("gkIntercepting: {}", gkIntercepting);

    if ((false) && (ballAriving) && (oneTouchNear) && (!assholeNear) && (!assholeHasBall) && (!ballMovingFast) &&
        (!ownAttackHasBall) && (!ballIsToGoal))
    {
        WaitForPass(robot_num, !oppGoalOpen);
    }
    else if ((interceptNear) && (!assholeHasBall) && (!ballMovingFast) && (!ownAttackHasBall) && (!ballIsToGoal) &&
             (!gkIntercepting) && (!stop))
    {
        ERRTSetObstacles(robot_num, 0, 1);
        // tech_circle(robot_num,Common::sign(m_world_state.ball.position.y)*side*60 ,0,15,false);
        tech_circle(robot_num,
                    m_world_state.ball.position.angleWith(
                        Common::Vec2(side * (Common::field().width + 110), 0)),
                    0, 500, true, 0, 0, 0);
    }
    else
    {
        ERRTSetObstacles(robot_num, stop, true);
        OwnRobot[robot_num].face(Common::Vec2((*defendTarget).x, (*defendTarget).y));
        ERRTNavigate2Point(robot_num, target, 100, stop ? VelocityProfile::Type::Aroom : VelocityProfile::Type::Mamooli);
    }
    // side = -side;
}
} // namespace Tyr::Soccer
