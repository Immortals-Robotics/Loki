#include "filtered.h"

namespace Tyr::Vision
{

void Filtered::processBalls()
{
    filterBalls();
}

void Filtered::newKalmanBall(const Common::Vec2 &t_position, const bool &t_seen)
{
    Eigen::VectorXd z(2);

    z(0) = t_position.x;
    z(1) = t_position.y;
    m_ball_ekf->process(z, t_seen, m_state.own_robot, m_state.opp_robot);
    auto m_x = m_ball_ekf->getSate();
    m_ball_ekf_future->process(z, t_seen, m_state.own_robot, m_state.opp_robot);
    auto future           = m_ball_ekf_future->getFutureState(200, m_state.own_robot, m_state.opp_robot);
    m_state.ball.position = Common::Vec2(m_x(0), m_x(1));
    m_state.ball.velocity = Common::Vec2(m_x(2), m_x(3));
}

void Filtered::filterBalls()
{
    auto &balls = m_raw_state.balls;

    int             id  = std::numeric_limits<int>::max();
    float           dis = std::numeric_limits<float>::max();
    Eigen::VectorXd z(2);

    for (size_t i = 0; i < balls.size(); i++)
    {
        const float curr_dis = balls[i].position.distanceTo(m_last_raw_ball.position);
        if (curr_dis < dis)
        {
            dis = curr_dis;
            id  = i;
        }
    }

    if (dis < Common::config().vision.max_ball_2_frame_dist)
    {
        estimateBallHeight(balls[id].position);
        m_last_raw_ball = balls[id];

        if (m_ball_not_seen > 0)
        {
            m_ball_kalman.initializePos(balls[id].position.xy());
        }

        m_ball_kalman.updatePosition(balls[id].position.xy());
        m_state.ball.position = m_ball_kalman.getPosition();
        m_state.ball.velocity = m_ball_kalman.getVelocity();

        m_ball_not_seen         = 0;
        m_state.ball.seen_state = Common::SeenState::Seen;
    }

    else
    {
        m_ball_not_seen++;

        if (m_ball_not_seen > Common::config().vision.max_ball_frame_not_seen)
        {
            if (balls.size() > 0)
            {
                m_last_raw_ball = balls[id];
                m_ball_kalman.initializePos(balls[id].position.xy());

                m_ball_kalman.updatePosition(balls[id].position.xy());
                m_state.ball.position = m_ball_kalman.getPosition();
                m_state.ball.velocity = m_ball_kalman.getVelocity();


                m_ball_not_seen         = 0;
                m_state.ball.seen_state = Common::SeenState::Seen;
            }
            else
            {
                m_state.ball.velocity = Common::Vec2{};

                m_last_raw_ball = Common::RawBallState();

                m_state.ball.seen_state = Common::SeenState::CompletelyOut;
            }
        }
        else
        {
            m_state.ball.seen_state = Common::SeenState::TemporarilyOut;
        }
    }
}

void Filtered::predictBall()
{
    m_state.ball.position /= 1000.0f;
    m_state.ball.velocity /= 1000.0f;

    float k       = 0.25f; // velocity derate every sec(units (m/s)/s)
    float tsample = (float) 1.0f / (float) Common::config().vision.vision_frame_rate;

    float t;
    if (m_state.ball.seen_state == Common::SeenState::TemporarilyOut)
        t = tsample;
    else
        t = kPredictSteps * tsample;

    float dist       = m_state.ball.velocity.length() * t - k * (t * t) / 2.0f;
    float vball_pred = m_state.ball.velocity.length() - k * t;

    // if speed turns out to be negative..it means that ball has stopped, so calculate that amount of
    // distance traveled
    if (vball_pred < 0)
    {
        vball_pred = 0.0f;
        dist       = m_state.ball.velocity.lengthSquared() * k / 2.0f;
        // i.e the ball has stopped, so take a newer vision data for the prediction
    }

    if (m_state.ball.velocity.length() > 0)
    {
        m_state.ball.velocity = m_state.ball.velocity.normalized() * vball_pred;
        m_state.ball.position += m_state.ball.velocity.normalized() * dist;
    }

    m_state.ball.velocity *= 1000.0f;
    m_state.ball.position *= 1000.0f;
}

double          ts = 0;
KickSolveResult Filtered::estimate3Offset(double offset)
{
    const double a  = 9810;
    const double pz = 0;



    if (m_ball_records.size() > 7)
    {
        Eigen::Vector3d f           = getCameraPos(0);
        Eigen::MatrixXd A           = Eigen::MatrixXd(m_ball_records.size() * 2, 3);
        int             row         = 0;
        float           time_offset = 0.0;
        float           time        = 0.0;
        Eigen::VectorXd b           = Eigen::VectorXd(m_ball_records.size() * 2);
        for (const auto &record : m_ball_records)
        {
            //            Common::debug().draw(Common::Circle(Common::Vec2(record.x(),record.y()) , 100),
            //            Common::Color::red());
            time_offset       = time + t_offset;
            Eigen::Vector2d g = record;
            A.row(row * 2) << f.z() * time_offset, 0, (g.x() - f.x()) * time_offset;
            A.row((row * 2) + 1) << 0, f.z() * time_offset, (g.y() - f.y()) * time_offset;
            b(row * 2) = ((0.5 * a * time_offset * time_offset * (g.x() - f.x())) + (g.x() * f.z())) -
                         (m_kick_position.x() * f.z()) - ((g.x() - f.x()) * pz);
            b(row * 2 + 1) = ((0.5 * a * time_offset * time_offset * (g.y() - f.y())) + (g.y() * f.z())) -
                             (m_kick_position.y() * f.z()) - ((g.y() - f.y()) * pz);
            time += 0.017;
            row++;
        }

        Eigen::HouseholderQR<Eigen::MatrixXd> qr(A);
        Eigen::VectorXd                       x;
        x = qr.solve(b);
        return KickSolveResult(x, (A * x - b).lpNorm<1>(), t_offset);
    }
    auto dummy = Eigen::VectorXd(1);
    dummy << -100000;
    return KickSolveResult(dummy, -1000000, -1000000);
}

double last_vel = 0;
bool captured = false;
void   Filtered::estimateBallHeight(const Common::Vec2 &t_position)
{

    auto now = std::chrono::high_resolution_clock::now();

    // Convert the time_point to microseconds since epoch
    auto now_us = std::chrono::time_point_cast<std::chrono::microseconds>(now);

    // Get the duration since epoch in microseconds
    auto      value        = now_us.time_since_epoch();
    long long microseconds = value.count();

    Common::logCritical("time diff: {}", microseconds - ts);
    ts = microseconds;

    for (int i = 0; i < 16; i++)
    {
        if (m_state.own_robot[i].seen_state == Common::SeenState::CompletelyOut)
        {
            continue;
        }
        const auto robot =
            Common::Robot(m_state.own_robot[i].position, Common::field().robot_radius, m_state.own_robot[i].angle);
        if (robot.canKick(t_position) && !m_kick_detected)
        {
            m_kick_position << t_position.x, t_position.y;
            m_ball_records.clear();
            m_ball_records.push_back(m_kick_position);
            m_can_kick      = true;
            m_kick_detected = false;

            break;
        }
    }

    if (m_can_kick && (t_position - Common::Vec2(m_kick_position(0), m_kick_position(1))).length() > 10)
    {
        m_kick_detected = true;
        m_can_kick      = false;
    }
    if (m_kick_detected &&
        (t_position - Common::Vec2(m_ball_records.back().x(), m_ball_records.back().y())).length() < 20)
    {
        m_kick_detected = false;
        ball_height = 0;
    }
    if (m_kick_detected)
    {
        m_ball_records.push_back(Eigen::Vector2d(t_position.x, t_position.y));
        if (m_ball_records.size() > 50)
        {
            m_ball_records.pop_front();
        }
    }

    double t_off = 0.05;
    double inc   = t_off / 2;

    while (inc > 1e-3)
    {
        auto l1_norm_neg = estimate3Offset(t_off - 1e-5);
        auto l1_norm_pos = estimate3Offset(t_off + 1e-5);

        if (l1_norm_pos.l1Error == -1000000 || l1_norm_neg.l1Error == -1000000)
        {
            return;
        }

        if (l1_norm_neg.l1Error > l1_norm_pos.l1Error)
        {
            t_off += inc;
        }
        else
        {
            t_off -= inc;
        }
        inc = inc / 2;
    }

    auto result = estimate3Offset(t_off);

    auto            vel = result.x;
    Eigen::Vector3d acc(0, 0, -0.5 * 9810);
    if(result.l1Error <= 100000 && captured == false)
    {
        ball_height += vel.z() * 0.016 - 0.5 * 9810 * 0.017 * 0.017;
        last_vel = vel.z();
        captured = true;
    }
    else
    {
        last_vel -= 9810*0.017;
        ball_height+=last_vel * 0.017 - 0.5*9810*0.017*0.017;
    }
    Common::logCritical("vel z {} ball height {} vel {} error {}", vel.z(), ball_height, (last_vel), result.l1Error);
    if (vel.norm() * vel.norm() < 100)
    {
        captured = false;
    }

    if (ball_height <= 0)
    {
        captured = false;
        ball_height = 0;
    }

    if (vel.z() < 0)
    {
        return;
    }
    Common::debug().draw(Common::Circle(t_position, ball_height / 10), Common::Color::blue());
}

} // namespace Tyr::Vision
