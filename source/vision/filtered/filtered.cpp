#include "filtered.h"

namespace Tyr::Vision
{
Filtered::Filtered()
{
    const std::filesystem::path fast_filter_path = std::filesystem::path{DATA_DIR} / "ball_filter_fast.txt";
    const std::filesystem::path slow_filter_path = std::filesystem::path{DATA_DIR} / "ball_filter_slow.txt";

    m_ball_kalman = FilteredObject{fast_filter_path, slow_filter_path};

    for (int color_idx = 0; color_idx < 2; ++color_idx)
    {
        for (int robot_idx = 0; robot_idx < Common::Setting::kMaxRobots; robot_idx++)
        {
            m_robot_not_seen[color_idx][robot_idx] = std::numeric_limits<int>::max() - 1;
            m_robot_kalman[color_idx][robot_idx]   = FilteredObject{fast_filter_path, slow_filter_path};
        }
    }

    m_client = std::make_unique<Common::NngClient>(Common::setting().raw_world_state_url);
    m_server = std::make_unique<Common::NngServer>(Common::setting().world_state_url);

    m_storage.open("filtered-state");
}

Filtered::~Filtered()
{
    m_storage.close();
}

bool Filtered::receive()
{
    Protos::Immortals::RawWorldState pb_state;
    if (!m_client->receive(&pb_state))
        return false;

    m_raw_state = Common::RawWorldState(pb_state);
    return true;
}

void Filtered::process()
{
    processBalls();
    processRobots();

    m_state.time = Common::TimePoint::now();
}

void Filtered::store()
{
    Protos::Immortals::WorldState pb_state{};
    m_state.fillProto(&pb_state);
    m_storage.store(m_state.time.timestamp(), pb_state);
}

bool Filtered::publish() const
{
    Protos::Immortals::WorldState pb_state{};
    m_state.fillProto(&pb_state);

    return m_server->send(pb_state);
}
} // namespace Tyr::Vision
