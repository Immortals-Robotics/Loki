#pragma once

#include "../kalman/ekf_3d.h"
#include "../kalman/filtered_object.h"
#include "ball_estimator.h"
namespace Tyr::Vision
{
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

    void processBalls(const bool t_new_kalman);
    void filterBalls(const bool t_new_kalman);
    void predictBall();
    void newKalmanBall(const Common::Vec2 &t_position, const bool &t_seen, const int &t_camera_id,
                       const ChipEstimator::Ball3D &t_ball_3d);

private:
    // TODO: move to settings
    static constexpr float kPredictSteps       = 7.0f;
    static constexpr int   kMaxRobotSubstitute = 60;

    // Don't add prediction to Ball or Opponents if both velocities are below this threshold
    static constexpr float kIgnorePrediction = 0.045f;

    // If the filtering process yields velocities above these values, reset the filter state
    // All these are in metres/sec
    static constexpr float kRobotErrorVelocity = 4500.0f;

    static constexpr int kMaxHistDraw = 200;

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

    std::unique_ptr<Ekf3D> m_ball_ekf;
    std::unique_ptr<Ekf3D> m_ball_ekf_future;

    std::unique_ptr<KickDetector>  m_kick_detector;
    std::unique_ptr<ChipEstimator> m_chip_estimator;
    std::deque<Common::Vec2>       m_ball_hist;
};
} // namespace Tyr::Vision
