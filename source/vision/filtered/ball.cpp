#include "filtered.h"

namespace Tyr::Vision
{

void Filtered::processBalls()
{
    auto &balls = m_raw_state.balls;
    if(balls.size())
    {
        newKalmanBall(balls[0].position);
        //    } else if(balls.size())
        //    {
        //        Common::Vec2 merge(0,0);
        //        for (const auto ball:balls)
        //        {
        //            merge +=ball.position;
        //        }
        //        merge/=balls.size();
        //        newKalmanBall(merge);
        //    }
        //    filterBalls();
        //    predictBall();
    }
}

void Filtered::newKalmanBall(const Common::Vec2 &t_position)
{
    Eigen::VectorXd z(2);

    z(0) = t_position.x;
    z(1) = t_position.y;
    m_ball_ekf->process(z);
    auto m_x = m_ball_ekf->getSate();
    m_ball_ekf_future->process(z);
    auto future = m_ball_ekf_future->getFutureState(50);
//
    m_state.ball.position = Common::Vec2(m_x(0), m_x(1));
    m_state.ball.velocity = Common::Vec2(m_x(2), m_x(3));

    Common::logCritical("what: {}", Common::Vec2(future(2), future(3)).length());

    Common::debug().draw(Common::Circle{Common::Vec2(future(0), future(1)), 40}, Common::Color::blue());
    m_ball_not_seen         = 0;
    m_state.ball.seen_state = Common::SeenState::Seen;

//    if()
}

void Filtered::filterBalls()
{
    auto &balls = m_raw_state.balls;

    int   id  = std::numeric_limits<int>::max();
    float dis = std::numeric_limits<float>::max();

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
} // namespace Tyr::Vision
