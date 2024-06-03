#pragma once
#include "../filtered/ball_estimator.h"

namespace Tyr::Vision
{
class Ekf3D
{
public:
    Ekf3D(double t_dt, double t_delay) : m_dt(t_dt), m_delay(t_delay)
    {
        m_A = Eigen::MatrixXd(4, 4);
        m_A << 1, 0, m_dt, 0, 0, 1, 0, m_dt, 0, 0, 1, 0, 0, 0, 0, 1;
        init(Common::Vec2{0., 0.});
    }

    static inline Eigen::VectorXd processCollisions(Common::BallState t_ball, const Common::RobotState t_own_robots[],
                                                    const Common::RobotState t_opp_robots[])
    {
        Eigen::VectorXd output(4);

        if (t_ball.velocity.length() < 200)
        {
            output << t_ball.position.x, t_ball.position.y, t_ball.velocity.x, t_ball.velocity.y;
            return output;
        }
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
            const auto obstacle = Common::Robot(
                robot.position, Common::field().robot_radius + Common::field().ball_radius - 1, robot.angle);
            if (obstacle.inside(t_ball.position))
            {
                auto ball_line = Common::LineSegment(t_ball.position, t_ball.position - t_ball.velocity * 4);

                auto intersect = Common::Line::fromSegment(ball_line).intersect(obstacle.getFrontLine());
                if (intersect.has_value() && ((intersect->x - obstacle.getFrontLine().start.x) *
                                                      (intersect->x - obstacle.getFrontLine().end.x) <=
                                                  1 &&
                                              (intersect->y - obstacle.getFrontLine().start.y) *
                                                      (intersect->y - obstacle.getFrontLine().end.y) <=
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

    inline void getOptimalProcessNoise(double t_delta_t, const double &t_error)
    {
        double sigma = std::sqrt((3.0 * t_error) / t_delta_t) / t_delta_t;
        double dt3   = (1.0 / 3.0) * t_delta_t * t_delta_t * t_delta_t * sigma * sigma;
        double dt2   = (1.0 / 2.0) * t_delta_t * t_delta_t * sigma * sigma;
        double dt1   = t_delta_t * sigma * sigma;
        m_Q << dt3, 0, dt2, 0, 0, dt3, 0, dt2, dt2, 0, dt1, 0, 0, dt2, 0, dt1;
    }

    inline void predict(double t_delta_t, const int &t_delay_queue_size)
    {
        if (t_delta_t == 0)
        {
            return;
        }
        // Predict state
        m_x = m_A * m_x;
        getOptimalProcessNoise(t_delta_t, 0.7);
        // Predict covariance
        m_P = m_A * m_P * m_A.transpose() + m_Q;

        if (m_x_delay_buffer.size() == t_delay_queue_size)
        {
            m_x_delay_buffer.pop_front();
            m_P_delay_buffer.pop_front();
        }

        m_x_delay_buffer.push_back(m_x);
        m_P_delay_buffer.push_back(m_P);
    }

    inline void update(const Eigen::VectorXd &t_z, const int &t_forward_steps)
    {
        Eigen::VectorXd x_delayed = m_x_delay_buffer.front();
        Eigen::MatrixXd P_delayed = m_P_delay_buffer.front();

        // Measurement matrix (Cimp)
        Eigen::MatrixXd H(2, 4);
        H << 1, 0, 0, 0, 0, 1, 0, 0;

        // Measurement prediction residual (Loss vector)
        Eigen::VectorXd y = t_z - H * x_delayed;

        // Measurement covariance
        Eigen::MatrixXd S = H * P_delayed * H.transpose() + m_R;

        // Kalman gain
        Eigen::MatrixXd K = P_delayed * H.transpose() * S.inverse();

        // Update state
        x_delayed = x_delayed + K * y;

        // Update covariance
        P_delayed = (m_I - K * H) * P_delayed;

        for (int i = 0; i < t_forward_steps; ++i)
        {
            x_delayed = m_A * x_delayed;
            P_delayed = m_A * P_delayed * m_A.transpose() + m_Q;
        }

        m_x = x_delayed;
        m_P = P_delayed;
    }

    inline void process(const Common::Vec3 &t_z, const Common::Vec3 &t_estimator_vel, const bool &t_seen,
                        const Common::RobotState t_own_robots[], const Common::RobotState t_opp_robots[])
    {
        int delay_steps = m_delay / m_dt;
        delay_steps     = std::max(delay_steps, 1);
        predict(m_dt, delay_steps);
        const Eigen::Vector2d z(t_z.x, t_z.y);

        if (!m_airborne && t_z.z > 0)
        {
            m_airborne = true;
            m_x(0)     = t_z.x;
            m_x(1)     = t_z.y;
            m_x(2)     = 0;
            m_x(3)     = 0;
        }

        m_predicted_height = t_z.z;
        m_predicted_v_z    = t_estimator_vel.z;

        if (t_z.z == 0)
        {
            m_airborne = false;
        }
        if (t_seen)
        {
            update(z, delay_steps - 1);
        }
        Common::BallState ball;
        ball.position.x = m_x(0);
        ball.position.y = m_x(1);
        ball.velocity.x = m_x(2);
        ball.velocity.y = m_x(3);
        if (t_z.z <= 150. && !t_seen)
        {
            m_x = Ekf3D::processCollisions(ball, t_own_robots, t_opp_robots);
        }
    }

    //// Have to come after process
    inline Common::Vec3 getFutureState(const int &t_steps, const Common::RobotState t_own_robots[],
                                       const Common::RobotState t_opp_robots[])
    {
        Eigen::VectorXd x = m_x;
        bool impact = false;
        for (int i = 0; i < t_steps; i++)
        {
            Common::BallState ball;
            x               = m_A * x;
            ball.position.x = x(0);
            ball.position.y = x(1);
            ball.velocity.x = x(2);
            ball.velocity.y = x(3);

            if (m_predicted_height < 150.)
            {
                x = Ekf3D::processCollisions(ball, t_own_robots, t_opp_robots);
            }
            if (m_predicted_height > 0)
            {
                m_predicted_height += m_predicted_v_z * m_dt - 0.5 * kG * m_dt * m_dt;
                m_predicted_v_z -= kG * m_dt;
            }
            else
            {
                m_predicted_v_z    = 0;
                m_predicted_height = 0;
                if (!impact)
                {
                    Common::debug().draw(ball.position);
                    impact = true;
                }
            }

            Common::debug().draw(Common::Circle{Common::Vec2(x(0), x(1)), 10}, Common::Color::black());
        }
        return Common::Vec3(x(0), x(1), m_predicted_height);
    }

    inline void init(Common::Vec2 t_pos)
    {
        m_x = Eigen::VectorXd(4);
        m_x << t_pos.x, t_pos.y, 0, 0;

        // Initialize covariance matrix
        m_P = Eigen::MatrixXd(4, 4);
        m_P.setIdentity();
        m_P *= 1000.; // High uncertainty in initial state

        // Process noise covariance
        m_Q = Eigen::MatrixXd(4, 4);
        m_Q.setZero();

        // Measurement noise covariance
        m_R = Eigen::MatrixXd(2, 2);
        m_R.setIdentity();
        m_R *= 100.; // Low measurement noise

        // Identity matrix
        m_I = Eigen::MatrixXd::Identity(4, 4);
        m_x_delay_buffer.clear();
        m_P_delay_buffer.clear();
    }

    double m_dt;
    double m_delay;

    std::deque<Eigen::VectorXd> m_x_delay_buffer;
    std::deque<Eigen::MatrixXd> m_P_delay_buffer;

    void getSate(Common::Vec3 *t_position, Common::Vec3 *t_velocity)
    {
        *t_position = Common::Vec3(m_x(0), m_x(1), m_predicted_height);
        *t_velocity = Common::Vec3(m_x(2), m_x(3), m_predicted_v_z);
    }

private:
    Eigen::VectorXd        m_x; // State vector
    Eigen::MatrixXd        m_P; // Covariance matrix
    Eigen::MatrixXd        m_Q; // Process noise covariance
    Eigen::MatrixXd        m_R; // Measurement noise covariance
    Eigen::MatrixXd        m_I; // Identity matrix
    Eigen::MatrixXd        m_A;
    bool                   m_airborne         = false;
    float                  m_predicted_height = 0;
    float                  m_predicted_v_z    = 0;
    static constexpr float kG                 = 9810;
};

} // namespace Tyr::Vision