#pragma once

#include "../kalman/filtered_object.h"
namespace Tyr::Vision
{
class Ekf
{
public:
    // Constructor
    Ekf(double t_dt, double t_delay) : m_dt(t_dt), m_delay(t_delay)
    {
        // Initialize state vector
        m_x = Eigen::VectorXd(4);
        m_x.setZero();

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
        m_R *= 10.; // Low measurement noise

        // Identity matrix
        m_I = Eigen::MatrixXd::Identity(4, 4);

        m_A << 1, 0, m_dt, 0, 0, 1, 0, m_dt, 0, 0, 1, 0, 0, 0, 0, 1;
    }


//    inline void proccessCollisions(const std::vector<Common::RobotState> t_robots)
//    {
//
//    }

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
        getOptimalProccessNoise(t_delta_t, 10.7);
        // Predict covariance
        m_P = m_A * m_P * m_A.transpose() + m_Q;

        m_x_delay_buffer.push_back(m_x);
        m_P_delay_buffer.push_back(m_P);
        if (m_x_delay_buffer.size() > t_delay_queue_size)
        {
            m_x_delay_buffer.pop_front();
            m_P_delay_buffer.pop_front();
        }
    }

    inline void update(const Eigen::VectorXd &t_z, const int & t_forward_steps)
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

        for (int i = 0; i < t_forward_steps; ++i) {
            x_delayed = m_A * x_delayed;
            P_delayed = m_A * P_delayed * m_A.transpose() + m_Q;
        }

        m_x = x_delayed;
        m_P = P_delayed;
    }

    inline void process(const Eigen::VectorXd &t_z)
    {
        int delay_steps = m_delay / m_dt;
        delay_steps = std::max(delay_steps, 1);
        predict(m_dt, delay_steps);
        update(t_z, delay_steps);
    }

    inline Eigen::VectorXd getFutureState(const int &t_steps)
    {
        Eigen::VectorXd x = m_x;
        for(int i = 0 ; i < t_steps ; i ++ ) {
            x = m_A*x;
        }
        return x;
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

    void processBalls();
    void filterBalls();
    void predictBall();
    void newKalmanBall(const Common::Vec2 &t_position);

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

    static constexpr double kBallMaxSpeed = 15000.;
    std::unique_ptr<Ekf>    m_ball_ekf;
    std::unique_ptr<Ekf>    m_ball_ekf_future;
};
} // namespace Tyr::Vision
