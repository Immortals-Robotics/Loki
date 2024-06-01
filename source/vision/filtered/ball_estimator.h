#pragma once
#include <common/common.h>
using namespace Immortals;

namespace Tyr::Vision
{
class KickDetector
{
public:
    KickDetector()  = default;
    ~KickDetector() = default;

    struct Kick
    {
        Kick(const Common::Vec2 &t_position, const Common::Vec2 &t_velocity, const Common::Angle &t_direction,
             const bool &t_is_kicked)
            : position(t_position), velocity(t_velocity), direction(t_direction), is_kicked(t_is_kicked)
        {}
        Kick()
        {
            is_kicked = false;
        }
        Common::Vec2  position;
        Common::Vec2  velocity;
        Common::Angle direction;
        bool          is_kicked;
    };

    struct BallTime
    {
        BallTime(const Common::TimePoint &t_time, const Common::RawBallState &t_ball) : time(t_time), ball(t_ball)
        {}
        Common::TimePoint    time;
        Common::RawBallState ball;
    };
    inline Kick detect(const Common::RawBallState &t_ball, const Common::TimePoint &t_time,
                       const Common::RobotState t_own_robots[], const Common::RobotState t_opp_robots[],
                       const double &t_threshold, const bool t_fast = true)
    {
        m_ball_records.push_back(BallTime(t_time, t_ball));
        Kick          kick;
        Common::Vec2  candidate_kick_pos(-1000000, -1000000);
        Common::Angle candidate_kick_angle;
        if (m_ball_records.size() > (t_fast ? kFastRecordNum : kSlowRecordNum))
        {
            m_ball_records.pop_front();
        }
        std::vector<Common::Robot> robots;

        for (auto i = 0; i < Common::Config::Common::kMaxRobots; i++)
        {
            if (t_own_robots->seen_state != Common::SeenState::CompletelyOut)
            {
                robots.push_back(
                    Common::Robot(t_own_robots[i].position, Common::field().robot_radius, t_own_robots[i].angle));
            }
            if (t_opp_robots->seen_state != Common::SeenState::CompletelyOut)
            {
                robots.push_back(
                    Common::Robot(t_opp_robots[i].position, Common::field().robot_radius, t_opp_robots[i].angle));
            }
        }

        for (auto const &robot : robots)
        {
            if (robot.canKick(t_ball.position.xy()))
            {
                candidate_kick_pos   = t_ball.position.xy();
                candidate_kick_angle = robot.angle;
                continue;
            }
        }

        if (m_ball_records.size() < (t_fast ? kFastRecordNum : kSlowRecordNum) ||
            candidate_kick_pos == Common::Vec2(-1000000, -1000000))
        {
            return kick;
        }

        std::vector<Common::Vec2> velocities;
        for (auto i = 0; i < m_ball_records.size() - 1; i++)
        {
            velocities.push_back((m_ball_records[i + 1].ball.position.xy() - m_ball_records[i].ball.position.xy()) *
                                 1000000. /
                                 (m_ball_records[i + 1].time.microseconds() - m_ball_records[i].time.microseconds()));
        }

        std::vector<double> velocity_magnitudes;
        for (const auto &velocity : velocities)
        {
            velocity_magnitudes.push_back(velocity.length());
        }

        std::vector<Common::Vec2> velocity_changes;
        for (auto i = 0; i < velocities.size() - 1; i++)
        {
            velocity_changes.push_back(velocities[i + 1] - velocities[i]);
        }

        std::vector<double> velocity_changes_magnitudes;
        for (const auto &change : velocity_changes)
        {
            velocity_changes_magnitudes.push_back(change.length());
        }

        const double mean_velocity =
            std::accumulate(velocity_magnitudes.begin(), velocity_magnitudes.end(), 0) / velocity_magnitudes.size();
        const double sq_sum      = std::inner_product(velocity_magnitudes.begin(), velocity_magnitudes.end(),
                                                      velocity_magnitudes.begin(), 0.0);
        const double ev_velocity = std::sqrt(sq_sum / velocity_magnitudes.size() - mean_velocity * mean_velocity);
        const double threshold   = mean_velocity + t_threshold * ev_velocity;

        const bool kick_detected =
            std::any_of(velocity_changes_magnitudes.begin(), velocity_changes_magnitudes.end(),
                        [threshold](double t_change_magnitude) { return t_change_magnitude > threshold; });

        Common::logCritical("mean {} sq sum {} ev {} thr {} change {}", mean_velocity, sq_sum, ev_velocity, threshold,
                            velocity_changes_magnitudes[0]);
        if (kick_detected)
        {
            kick = Kick(candidate_kick_pos, velocities.back(), candidate_kick_angle, true);
        }
        return kick;
    }

private:
    std::deque<BallTime> m_ball_records;

    constexpr static int kFastRecordNum = 3;
    constexpr static int kSlowRecordNum = 5;
};

class ChipEstimator
{
public:
    //    struct KickSolveResult
    //    {
    //        Eigen::VectorXd x;
    //        double          l1Error;
    //        double          t_offset;
    //        KickSolveResult(Eigen::VectorXd t_x, double t_l1Error, double t_offset)
    //            : x(t_x), l1Error(t_l1Error), t_offset(t_offset){};
    //    };

    //    inline KickSolveResult estimate3Offset(double t_offset)
    //    {}

    //    inline void getBall3D(const Common::RawBallState &t_ball, const Common::RobotState t_own_robots[],
    //                          const Common::RobotState t_opp_robots[])
    //    {
    //        std::vector<Common::Robot> robots;
    //        //        for (int i = 0; i < Common::Config::Common::kMaxRobots; i++)
    //        //        {
    //        //            robots.push_back(Common::Robot(m_state.t_own_robots[i].position,
    //        Common::field().robot_radius,
    //        //                                           m_state.t_own_robots[i].angle));
    //        //            robots.push_back(Common::Robot(m_state.t_opp_robots[i].position,
    //        Common::field().robot_radius,
    //        //                                           m_state.t_opp_robots[i].angle));
    //        //        }
    //        //        for (const auto &robot : robots)
    //        //        {
    //        //            m_kick_position << t_position.x, t_position.y;
    //        //            m_ball_records.clear();
    //        //            m_ball_records.push_back(m_kick_position);
    //        //            m_can_kick      = true;
    //        //            m_kick_detected = false;
    //        //
    //        //            break;
    //        //        }
    //    }

    // private:
    //     std::deque<Eigen::Vector2d> m_ball_records;
    //     Eigen::Vector2d             m_kick_position;
    //     bool                        m_kick_detected = false;
    //     bool                        m_can_kick      = false;

    constexpr static float kG = 9810;
};
} // namespace Tyr::Vision