#pragma once

#include "../kalman/filtered_object.h"
#include "sector.h"
namespace Tyr::Vision
{
class Ekf
{
public:
    // Constructor
    Ekf(double t_dt, double t_delay) : m_dt(t_dt), m_delay(t_delay)
    {
        m_A = Eigen::MatrixXd(4, 4);
        m_A << 1, 0, m_dt, 0, 0, 1, 0, m_dt, 0, 0, 1, 0, 0, 0, 0, 1;
        init(Common::Vec2{0., 0.});
    }

    static Eigen::VectorXd processCollisions(Common::BallState t_ball, const Common::RobotState t_own_robots[],
                                             const Common::RobotState t_opp_robots[]);

    inline void getOptimalProccessNoise(double t_delta_t, const double &t_error)
    {
        double sigma = std::sqrt((3.0 * t_error) / t_delta_t) / t_delta_t;
        double dt3   = (1.0 / 3.0) * t_delta_t * t_delta_t * t_delta_t * sigma * sigma;
        double dt2   = (1.0 / 2.0) * t_delta_t * t_delta_t * sigma * sigma;
        double dt1   = t_delta_t * sigma * sigma;
        m_Q << dt3, 0, dt2, 0, 0, dt3, 0, dt2, dt2, 0, dt1, 0, 0, dt2, 0, dt1;
    }
    // Predict and update methods
    inline void predict(double t_delta_t, const int &t_delay_queue_size)
    {
        if (t_delta_t == 0)
        {
            return;
        }
        // Predict state
        m_x = m_A * m_x;
        getOptimalProccessNoise(t_delta_t, 0.7);
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

    inline void process(const Eigen::VectorXd &t_z, const bool &t_seen, const Common::RobotState t_own_robots[],
                        const Common::RobotState t_opp_robots[])
    {
        int delay_steps = m_delay / m_dt;
        delay_steps     = std::max(delay_steps, 1);
        predict(m_dt, delay_steps);
        if (t_seen)
        {
            update(t_z, delay_steps - 1);
        }
        Common::BallState ball;
        ball.position.x = m_x(0);
        ball.position.y = m_x(1);
        ball.velocity.x = m_x(2);
        ball.velocity.y = m_x(3);
        m_x             = Ekf::processCollisions(ball, t_own_robots, t_opp_robots);
    }

    inline Eigen::VectorXd getFutureState(const int &t_steps, const Common::RobotState t_own_robots[],
                                          const Common::RobotState t_opp_robots[])
    {
        Eigen::VectorXd x = m_x;
        for (int i = 0; i < t_steps; i++)
        {
            Common::BallState ball;
            x               = m_A * x;
            ball.position.x = x(0);
            ball.position.y = x(1);
            ball.velocity.x = x(2);
            ball.velocity.y = x(3);
            x               = Ekf::processCollisions(ball, t_own_robots, t_opp_robots);
            Common::debug().draw(Common::Circle{Common::Vec2(x(0), x(1)), 20}, Common::Color::blue());
        }
        return x;
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

    Eigen::VectorXd getSate() const
    {
        return m_x;
    }

private:
    Eigen::VectorXd m_x; // State vector
    Eigen::MatrixXd m_P; // Covariance matrix
    Eigen::MatrixXd m_Q; // Process noise covariance
    Eigen::MatrixXd m_R; // Measurement noise covariance
    Eigen::MatrixXd m_I; // Identity matrix
    Eigen::MatrixXd m_A;
};
struct KickSolveResult
{
    Eigen::VectorXd x;
    double          l1Error;
    double          t_offset;
    KickSolveResult(Eigen::VectorXd t_x, double t_l1Error, double t_offset)
        : x(t_x), l1Error(t_l1Error), t_offset(t_offset){};
};
class Filtered
{
public:
    Filtered();
    ~Filtered() = default;

    bool receive();
    void process();

    bool publish() const;

private:
    void processRobots();
    void filterRobots(Common::TeamColor t_color);
    void predictRobots();
    void sendStates();

    void            processBalls();
    void            filterBalls();
    void            predictBall();
    void            newKalmanBall(const Common::Vec2 &t_position, const bool &t_seen);
    Eigen::Vector3d getCameraPos(const int t_id);
    void            estimateBallHeight(const Common::Vec2 &t_position);



    KickSolveResult estimate3Offset(double t_offset);
    KickSolveResult estimate5Offset(double t_offset);

private:
    // TODO: move to settings
    static constexpr float kPredictSteps       = 7.0f;
    static constexpr int   kMaxRobotSubstitute = 60;

    // Don't add prediction to Ball or Opponents if both velocities are below this threshold
    static constexpr float kIgnorePrediction = 0.045f;

    // If the filtering process yields velocities above these values, reset the filter state
    // All these are in metres/sec
    static constexpr float kRobotErrorVelocity = 4500.0f;

    std::unique_ptr<Common::NngClient> m_client;
    std::unique_ptr<Common::NngServer> m_server;

    Common::RawBallState m_last_raw_ball; // The last position of the locked ball
    FilteredObject       m_ball_kalman;
    int                  m_ball_not_seen = std::numeric_limits<int>::max() - 1;

    FilteredObject m_robot_kalman[2][Common::Config::Common::kMaxRobots];
    int            m_robot_not_seen[2][Common::Config::Common::kMaxRobots];

    Common::MedianFilter<Common::Angle> m_angle_filter[2][Common::Config::Common::kMaxRobots];
    Common::Angle                       m_raw_angles[2][Common::Config::Common::kMaxRobots];

    Common::RawWorldState m_raw_state;
    Common::WorldState    m_state;

    std::unique_ptr<Ekf> m_ball_ekf;
    std::unique_ptr<Ekf> m_ball_ekf_future;

    std::deque<Eigen::Vector2d> m_ball_records;
    Eigen::Vector2d             m_kick_position;
    bool                        m_kick_detected = false;
    bool                        m_can_kick      = false;
    double ball_height = 0;
};
} // namespace Tyr::Vision
