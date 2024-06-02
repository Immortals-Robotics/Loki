#include "filtered.h"

namespace Tyr::Vision
{

void Filtered::processBalls(const bool t_new_kalman)
{
    filterBalls(t_new_kalman);
    if (!t_new_kalman)
    {
        predictBall();
    }
}

std::deque<Common::Vec2> ball_hist;
void Filtered::newKalmanBall(const Common::Vec2 &t_position, const bool &t_seen, const int &t_camera_id,
                             const ChipEstimator::Ball3D &t_ball_3d = ChipEstimator::Ball3D{Common::Vec2(0,0)})
{
    Eigen::VectorXd z(2);

    z(0) = t_position.x;
    z(1) = t_position.y;

    ball_hist.push_back(t_position);

    if(ball_hist.size() > 1000) {
        ball_hist.pop_front();
    }

    if(m_state.ball.velocity.length() < 10) {
        ball_hist.clear();
    }

    for(const auto & p: ball_hist) {
        Common::debug().draw(Common::Circle(p, 21),Common::Color::blue());
    }
    Eigen::Vector3d z_3d;
    z_3d << t_ball_3d.position.x, t_ball_3d.position.y, t_ball_3d.position.z;

    m_ball_ekf->process(z, t_seen, m_state.own_robot, m_state.opp_robot, z_3d);
    auto m_x = m_ball_ekf->getSate();

    m_ball_ekf_future->process(z, t_seen, m_state.own_robot, m_state.opp_robot, z_3d);

//    if (t_ball_3d.position.z > 150.)
//    {
//        m_ball_ekf_future->getFutureState(200, m_state.own_robot, m_state.opp_robot, true);
//    }
//    else
//    {
//        m_ball_ekf_future->getFutureState(200, m_state.own_robot, m_state.opp_robot, false);
//    }


    if (t_ball_3d.position.z> 0)
    {
        m_state.ball.position = ChipEstimator::projectToGround(Common::Vec3(m_x(0), m_x(1), t_ball_3d.position.z - 0.032*9810),
                                                               ChipEstimator::getCameraPos(t_camera_id));
        m_state.ball.velocity = ChipEstimator::projectToGround(Common::Vec3(m_x(2), m_x(3), t_ball_3d.velocity.z),
                                                               ChipEstimator::getCameraPos(t_camera_id));

        Common::debug().draw(Common::Circle(m_state.ball.position, Common::field().ball_radius + t_ball_3d.position.z / 10),
                             Common::Color::red());
    }
    else
    {
        m_state.ball.position = Common::Vec2(m_x(0), m_x(1));
        m_state.ball.velocity = Common::Vec2(m_x(2), m_x(3));
    }
}

void Filtered::filterBalls(const bool t_new_kalman)
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
        ChipEstimator::Ball3D ball_3d =
            m_chip_estimator->getBall3D(balls[id].position.xy(), m_raw_state.frames[balls[id].m_frame_idx].camera_id,
                                        m_raw_state.time, m_state.own_robot, m_state.opp_robot);
        m_last_raw_ball = balls[id];
        z << balls[id].position.x, balls[id].position.y;
        if (m_ball_not_seen > 0)
        {

            if (t_new_kalman)
            {
                m_ball_ekf->init(balls[id].position.xy());
            }
            else
            {
                m_ball_kalman.initializePos(balls[id].position.xy());
            }
        }

        if (t_new_kalman)
        {
            newKalmanBall(balls[id].position.xy(), true, m_raw_state.frames[balls[id].m_frame_idx].camera_id,
                          ball_3d);
        }
        else
        {
            m_ball_kalman.updatePosition(balls[id].position.xy());
            m_state.ball.position = m_ball_kalman.getPosition();
            m_state.ball.velocity = m_ball_kalman.getVelocity();
        }
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
                if (t_new_kalman)
                {
                    m_ball_ekf->init(balls[id].position.xy());
                    newKalmanBall(balls[id].position.xy(), true, m_raw_state.frames[balls[id].m_frame_idx].camera_id);
                }
                else
                {
                    m_ball_kalman.initializePos(balls[id].position.xy());

                    m_ball_kalman.updatePosition(balls[id].position.xy());
                    m_state.ball.position = m_ball_kalman.getPosition();
                    m_state.ball.velocity = m_ball_kalman.getVelocity();
                }
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
            if (t_new_kalman)
            {
                newKalmanBall(Common::Vec2(0, 0), false, m_raw_state.frames[balls[id].m_frame_idx].camera_id);
            }
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
