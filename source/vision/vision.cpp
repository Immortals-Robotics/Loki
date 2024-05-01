#include "vision.h"

namespace Tyr::Vision
{
Vision::Vision(Common::WorldState *_State)
{
    m_state = _State;

    // Initializing the settings:
    our_color = Common::setting().our_color == Common::TeamColor::Yellow ? COLOR_YELLOW : COLOR_BLUE;
    our_side  = Common::setting().our_side == Common::TeamSide::Left ? LEFT_SIDE : RIGHT_SIDE;

    for (int i = 0; i < Common::Setting::kCamCount; i++)
        packet_recieved[i] = false;

    lastRawBall.set_x(0.0f);
    lastRawBall.set_y(0.0f);

    std::string fast_filter_path(DATA_DIR);
    fast_filter_path.append("/ball_filter_fast.txt");
    std::string slow_filter_path(DATA_DIR);
    slow_filter_path.append("/ball_filter_slow.txt");

    ball_kalman.initialize(fast_filter_path.c_str(), slow_filter_path.c_str());

    for (int i = 0; i < Common::Setting::kMaxRobots; i++)
    {
        robot_kalman[0][i].initialize(fast_filter_path.c_str(), slow_filter_path.c_str());
        robot_kalman[1][i].initialize(fast_filter_path.c_str(), slow_filter_path.c_str());
        rawAngles[0][i] = 0.0f;
        rawAngles[1][i] = 0.0f;
    }

    ball_kalman.initialize(fast_filter_path.c_str(), slow_filter_path.c_str());

    // Launching UDP Connections
    if (!connectToVisionServer())
    {
        std::cout << "Failed to connect to Vision UDP" << std::endl;
    }
}
Vision::~Vision()
{}

void Vision::recieveAllCameras()
{
    if (!isConnected())
    {
        std::cout << "	Hey you! Put the LAN cable back in its socket, or ..." << std::endl;
        return;
        // connectToVisionServer ( setting -> UDP_Adress , setting -> LocalPort );
    }

    bool cams_ready = false;
    while (!cams_ready)
    {
        cams_ready = true;
        for (int i = 0; i < Common::Setting::kCamCount; i++)
        {
            bool new_cam_ready = packet_recieved[i] || (!Common::setting().use_camera[i]);
            if (!new_cam_ready)
            {
                cams_ready = false;
                break;
            }
        }
        if (cams_ready)
            break;
        // std::cout << "bodo dg    " << cams_ready << std::endl;
        recievePacket();
    }
}

void Vision::ProcessVision()
{
    ProcessBalls();
    ProcessRobots();
    ProcessParam();

    for (int i = 0; i < Common::Setting::kCamCount; i++)
        packet_recieved[i] = false;
}
} // namespace Tyr::Vision