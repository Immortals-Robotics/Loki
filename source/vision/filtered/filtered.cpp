#include "filtered.h"

namespace Tyr::Vision
{
Filtered::Filtered()
{
    const std::filesystem::path fast_filter_path = std::filesystem::path{DATA_DIR} / "ball_filter_fast.txt";
    const std::filesystem::path slow_filter_path = std::filesystem::path{DATA_DIR} / "ball_filter_slow.txt";

    m_ball_kalman = FilteredObject{fast_filter_path, slow_filter_path};

    for (int color_idx = 0; color_idx < 2; ++color_idx)
    {
        for (int robot_idx = 0; robot_idx < Common::Config::Common::kMaxRobots; robot_idx++)
        {
            m_robot_not_seen[color_idx][robot_idx] = std::numeric_limits<int>::max() - 1;
            m_robot_kalman[color_idx][robot_idx]   = FilteredObject{fast_filter_path, slow_filter_path};
        }
    }

    m_client          = std::make_unique<Common::NngClient>(Common::config().network.raw_world_state_url);
    m_server          = std::make_unique<Common::NngServer>(Common::config().network.world_state_url);
    m_ball_ekf        = std::make_unique<Ekf>(1. / 60., 0.01);
    m_ball_ekf_future = std::make_unique<Ekf>(1. / 60., 0.01);
    m_kick_detector   = std::make_unique<KickDetector>();
    m_chip_estimator  = std::make_unique<ChipEstimator>();
}

bool Filtered::receive()
{
    Protos::Immortals::RawWorldState pb_state;
    if (!m_client->receive(&pb_state))
        return false;

    m_raw_state = Common::RawWorldState(pb_state);
    return true;
}

void Filtered::process()
{
    processBalls(true);
    processRobots();

    m_state.time = Common::TimePoint::now();
}

bool Filtered::publish() const
{
    Protos::Immortals::WorldState pb_state{};
    m_state.fillProto(&pb_state);

    return m_server->send(m_state.time, pb_state);
}

Eigen::VectorXd Ekf::processCollisions(Common::BallState t_ball, const Common::RobotState t_own_robots[],
                                       const Common::RobotState t_opp_robots[])
{
    Eigen::VectorXd output(4);

    Common::BallState predicted_ball = t_ball;
    float             ball_radius    = Common::field().ball_radius;

    std::vector<Common::RobotState> robots;
    for (int i = 0; i < Common::Config::Common::kMaxRobots; i++)
    {
        robots.push_back(t_opp_robots[i]);
        robots.push_back(t_own_robots[i]);
    }

    for (const auto &robot : robots)
    {
        if (robot.seen_state == Common::SeenState::CompletelyOut)
        {
            continue;
        }
        const auto obstacle =
            Common::Robot(robot.position, Common::field().robot_radius + Common::field().ball_radius, robot.angle);
        if (obstacle.inside(t_ball.position))
        {
            auto ball_line = Common::LineSegment(t_ball.position, t_ball.position - t_ball.velocity * 4);

            auto intersect = Common::Line::fromSegment(ball_line).intersect(obstacle.getFrontLine());
            if (intersect.has_value() &&
                ((intersect->x - obstacle.getFrontLine().start.x) * (intersect->x - obstacle.getFrontLine().end.x) <=
                     1 &&
                 (intersect->y - obstacle.getFrontLine().start.y) * (intersect->y - obstacle.getFrontLine().end.y) <=
                     1))
            {
                auto collision_angle = robot.angle - (t_ball.velocity).toAngle();
                if (collision_angle.deg360() > 90 && collision_angle.deg360() < 270)
                {
                    // 0.5 should be tuned
                    auto normal_vec = robot.angle.toUnitVec();

                    auto v_n = t_ball.velocity.dot(normal_vec);
                    auto v_t = t_ball.velocity - (normal_vec * v_n);
                    /// damping factor
                    v_n                     = v_n * (1 - 0.98);
                    v_t                     = v_t * 0.1;
                    auto new_vel            = (normal_vec * v_n) * -1 + v_t;
                    predicted_ball.position = *intersect;
                    predicted_ball.velocity = new_vel;
                    if (predicted_ball.velocity.length())
                    {
                        predicted_ball.position += (*intersect - robot.position).normalized() * (ball_radius * 2);
                    }
                    break;
                }
            }

            auto circle_intersect = Common::Line::fromSegment(ball_line).intersect(
                Common::Circle{robot.position, Common::field().robot_radius + ball_radius});
            if (circle_intersect.size())
            {
                auto reflect_point = circle_intersect.at(0);
                if (circle_intersect.size() > 1)
                {
                    if (circle_intersect.at(1).distanceTo(t_ball.position - t_ball.velocity) <
                        circle_intersect.at(0).distanceTo(t_ball.position - t_ball.velocity))
                    {
                        reflect_point = circle_intersect.at(1);
                    }
                }

                auto normal_vec = (reflect_point - robot.position).normalized();

                auto v_n = t_ball.velocity.dot(normal_vec);
                auto v_t = t_ball.velocity - (normal_vec * v_n);
                /// damping factor
                v_n          = v_n * (1 - 0.9);
                v_t          = v_t * 0.4;
                auto new_vel = (normal_vec * v_n) * -1 + v_t;

                predicted_ball.velocity = new_vel;
                predicted_ball.position = reflect_point;
                if (predicted_ball.velocity.length())
                {
                    predicted_ball.position += normal_vec * (ball_radius * 2);
                }
            }

            break;
        }
    }

    output << predicted_ball.position.x, predicted_ball.position.y, predicted_ball.velocity.x,
        predicted_ball.velocity.y;
    return output;
}

} // namespace Tyr::Vision
