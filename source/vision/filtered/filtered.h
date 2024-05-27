#pragma once

#include "../kalman/filtered_object.h"
namespace Tyr::Vision
{
class Ekf {
public:
    // Constructor
    Ekf(double dt, double delay): m_dt(dt), m_delay(delay)
    {
        // Initialize state vector
        m_x = Eigen::VectorXd(6);
        m_x.setZero();

        // Initialize covariance matrix
        m_P = Eigen::MatrixXd(6, 6);
        m_P.setIdentity();
        m_P *= 100;  // High uncertainty in initial state

        // Process noise covariance
        m_Q = Eigen::MatrixXd(6, 6);
        m_Q.setZero();
        m_Q(0, 0) = m_Q(1, 1) = 1.1;  // Position noise
        m_Q(2, 2) = m_Q(3, 3) = 1.1;  // Velocity noise
        m_Q(4, 4) = m_Q(5, 5) = 1.1;  // Acceleration noise

        // Measurement noise covariance
        m_R = Eigen::MatrixXd(2, 2);
        m_R.setIdentity();
        m_R *= 0.01;  // Low measurement noise

        // Identity matrix
        m_I = Eigen::MatrixXd::Identity(6, 6);
    }

    // Predict and update methods
    inline void predict(double delta_t) {
        Eigen::MatrixXd A(6, 6);
        A << 1, 0, delta_t, 0, 0.5 * delta_t * delta_t, 0,
            0, 1, 0, delta_t, 0, 0.5 * delta_t * delta_t,
            0, 0, 1, 0, delta_t, 0,
            0, 0, 0, 1, 0, delta_t,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

        // Predict state
        m_x = A * m_x;

        // Predict covariance
        m_P = A * m_P * A.transpose() + m_Q;
    }
    inline void update(const Eigen::VectorXd &z, double delta_t) {
        // Predict state to the measurement time (compensate for delay)
        predict(delta_t);

        // Measurement matrix (Cimp)
        Eigen::MatrixXd H(2, 6);
        H << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0;

        // Measurement prediction residual (Loss vector)
        Eigen::VectorXd y = z - H * m_x;

        // Measurement covariance
        Eigen::MatrixXd S = H * m_P * H.transpose() + m_R;

        // Kalman gain
        Eigen::MatrixXd K = m_P * H.transpose() * S.inverse();

        // Update state
        m_x = m_x + K * y;

        // Update covariance
        m_P = (m_I - K * H) * m_P;

        // Propagate state back to the current time
        predict(-delta_t);
    }

    double m_dt;          // Time step for 60 Hz
    double m_delay;

    Eigen::VectorXd getSate() const {
        return m_x;
    }
private:
    Eigen::VectorXd m_x;  // State vector
    Eigen::MatrixXd m_P;  // Covariance matrix
    Eigen::MatrixXd m_Q;  // Process noise covariance
    Eigen::MatrixXd m_R;  // Measurement noise covariance
    Eigen::MatrixXd m_I;  // Identity matrix
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

    std::unique_ptr<Ekf> m_ball_ekf;

};
} // namespace Tyr::Vision
