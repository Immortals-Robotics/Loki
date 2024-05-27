#include "plot.h"

namespace Tyr::Gui
{
PlotMenu::PlotMenu()
{
    ImPlot::CreateContext();
}

PlotMenu::~PlotMenu()
{
    ImPlot::DestroyContext();
}

void PlotMenu::pushData(const Common::WorldState &t_state, const bool &t_playback)
{
    if (m_last_time == t_state.time)
    {
        return;
    }

    if (std::abs((t_state.time - m_last_time).seconds()) > 0.2f)
    {
        m_data.clear();
    }

    if (m_data.size() >= kPlotQueueSize)
        m_data.pop_front();
    m_data.emplace_back(t_state);

    m_last_time = t_state.time;

    if (t_playback && m_data.size() > 1)
    {
        std::sort(m_data.begin(), m_data.end(),
                  [](const Common::WorldState &t_first, const Common::WorldState &t_second)
                  { return t_first.time < t_second.time; });
    }
}

void PlotMenu::draw(const Common::WorldState &t_world, const bool &t_playback)
{
    pushData(t_world, t_playback);

    if (ImGui::BeginTable("plot-setting", 3, ImGuiTableFlags_BordersInnerV))
    {
        const char *item_choices[] = {"Our Robot", "Opp Robot", "Ball"};

        ImGui::TableNextColumn();
        ImGui::Combo("Item", reinterpret_cast<int *>(&m_item), item_choices, IM_ARRAYSIZE(item_choices));

        if (m_item == Item::OurRobot || m_item == Item::OppRobot)
        {
            const char *id_choices[] = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "A", "B", "C", "D", "E", "F"};

            ImGui::TableNextColumn();
            ImGui::Combo("ID", &m_id, id_choices, IM_ARRAYSIZE(id_choices));
        }

        const char *data_choices[] = {"vel", "vel xy"};

        ImGui::TableNextColumn();
        ImGui::Combo("Data", reinterpret_cast<int *>(&m_type), data_choices, IM_ARRAYSIZE(data_choices));

        ImGui::EndTable();
    }

    ImGui::Separator();

    if (ImPlot::BeginPlot("Plot", ImVec2(-1, ImGui::GetWindowHeight() - 60.), ImPlotFlags_NoTitle))
    {
        auto getter_x = [](const int t_idx, void *t_user_data)
        {
            const PlotMenu &menu = *static_cast<PlotMenu *>(t_user_data);

            const auto [time, data] = menu.data(t_idx);
            return ImPlotPoint(time.seconds(), data.x);
        };

        auto getter_y = [](const int t_idx, void *t_user_data)
        {
            const PlotMenu &menu = *static_cast<PlotMenu *>(t_user_data);

            const auto [time, data] = menu.data(t_idx);
            return ImPlotPoint(time.seconds(), data.y);
        };

        auto getter_len = [](const int t_idx, void *t_user_data)
        {
            const PlotMenu &menu = *static_cast<PlotMenu *>(t_user_data);

            const auto [time, data] = menu.data(t_idx);
            return ImPlotPoint(time.seconds(), data.length());
        };

        ImPlot::SetupAxes("time (s)", "Vel (mm/s)", ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit);
        if (t_playback)
        {
            ImPlot::SetupAxisLimitsConstraints(ImAxis_X1, 0, INFINITY);
        }
        else
        {
            ImPlot::SetupAxisLimitsConstraints(ImAxis_X1, 0, 30);
        }
        ImPlot::SetupAxisZoomConstraints(ImAxis_X1, 3., INFINITY);

        switch (m_type)
        {
        case Type::XY:
            ImPlot::PlotLineG("Vel x", getter_x, this, m_data.size());
            ImPlot::PlotLineG("Vel y", getter_y, this, m_data.size());
            break;
        case Type::Vel:
            ImPlot::PlotLineG("Vel", getter_len, this, m_data.size());
            break;
        default:
            break;
        }
        ImPlot::EndPlot();
    }
}
} // namespace Tyr::Gui
