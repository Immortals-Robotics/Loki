#include "raw.h"

namespace Tyr::Vision
{
Raw::Raw()
{
    for (int i = 0; i < Common::Setting::kCamCount; i++)
        m_packet_received[i] = false;

    m_client = std::make_unique<Common::UdpClient>(Common::setting().vision_address);
    if (!m_client->isConnected())
    {
        Common::logCritical("Failed to connect to Vision UDP");
    }

    m_server = std::make_unique<Common::NngServer>(Common::setting().raw_world_state_url);
}

void Raw::process()
{
    Protos::SSL_DetectionFrame merged_frame;
    for (int i = 0; i < Common::Setting::kCamCount; i++)
    {
        if (Common::setting().use_camera[i])
        {
            merged_frame.MergeFrom(m_d_frame[i]);
        }
    }

    m_state = Common::RawWorldState(merged_frame);

    processBalls();
    processRobots();

    m_state.time = Common::TimePoint::now();

    for (int i = 0; i < Common::Setting::kCamCount; i++)
        m_packet_received[i] = false;
}

bool Raw::publish() const
{
    Protos::Immortals::RawWorldState pb_state{};
    m_state.fillProto(&pb_state);

    return m_server->send(m_state.time, pb_state);
}
} // namespace Tyr::Vision
