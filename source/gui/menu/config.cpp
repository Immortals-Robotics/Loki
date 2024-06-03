#include "config.h"

namespace Tyr::Gui
{
void ConfigMenu::drawItem(const std::string &t_key, toml::node &t_value)
{
    static int depth = 0;

    depth++;
    ImGui::PushID(t_key.c_str());

    if (t_value.is_table())
    {
        // only put root-level tables inside a tree node
        const bool tree = depth == 1;

        if (!tree || ImGui::TreeNode(t_key.c_str()))
        {
            if (!tree)
                ImGui::SeparatorText(t_key.c_str());

            drawTable(*t_value.as_table());

            if (tree)
                ImGui::TreePop();
        }
    }
    else if (t_value.is_array())
    {
        int columns = 0;
        if (t_key == "use_camera")
            columns = 4;

        if (ImGui::TreeNode(t_key.c_str()))
        {
            drawArray(*t_value.as_array(), columns);
            ImGui::TreePop();
        }
    }
    else if (t_value.is_string())
    {
        ImGui::SetNextItemWidth(100);
        m_dirty |= ImGui::InputText(t_key.c_str(), &t_value.as_string()->get());
    }
    else if (t_value.is_floating_point())
    {
        ImGui::SetNextItemWidth(100);
        m_dirty |= ImGui::InputScalar(t_key.c_str(), ImGuiDataType_Double, &t_value.as_floating_point()->get());
    }
    else if (t_value.is_integer())
    {
        // handle special cases
        if (t_key == "our_side")
        {
            static constexpr std::array kEnums = {"Left", "Right"};
            static_assert(static_cast<int>(Common::TeamSide::Left) == 0);
            static_assert(static_cast<int>(Common::TeamSide::Right) == 1);

            m_dirty |= ImGui::Combo(t_key.c_str(), reinterpret_cast<int *>(&t_value.as_integer()->get()),
                                            kEnums.data(), kEnums.size());
        }
        else if (t_key == "our_color")
        {
            static constexpr std::array kEnums = {"Blue", "Yellow"};
            static_assert(static_cast<int>(Common::TeamColor::Blue) == 0);
            static_assert(static_cast<int>(Common::TeamColor::Yellow) == 1);

            m_dirty |= ImGui::Combo(t_key.c_str(), reinterpret_cast<int *>(&t_value.as_integer()->get()),
                                            kEnums.data(), kEnums.size());
        }
        else
        {
            ImGui::SetNextItemWidth(100);
            m_dirty |= ImGui::InputScalar(t_key.c_str(), ImGuiDataType_S64, &t_value.as_integer()->get());
        }
    }
    else if (t_value.is_boolean())
    {
        m_dirty |= ImGui::Checkbox(t_key.c_str(), &t_value.as_boolean()->get());
    }
    else
    {
        Common::logWarning("Config item {} has unsupported config type: {}", t_key, (int) t_value.type());
    }

    depth--;
    ImGui::PopID();
}

void ConfigMenu::drawArray(toml::array &t_array, const int t_columns)
{
    if (t_columns == 0 || ImGui::BeginTable("array", t_columns))
    {
        t_array.for_each(
            [this, t_columns](const int idx, auto &&value)
            {
                if (t_columns)
                    ImGui::TableNextColumn();
                drawItem(std::to_string(idx), value);
            });

        if (t_columns)
            ImGui::EndTable();
    }
}

void ConfigMenu::drawTable(toml::table &t_table)
{
    for (auto &&[key, value] : t_table)
    {
        drawItem(std::string{key.str()}, value);
    }
}

void ConfigMenu::draw()
{
    drawTable(Common::config().root());

    if (m_dirty)
    {
        Common::Services::config().save();
        Common::Services::config().load();

        m_dirty = false;
    }
}
} // namespace Tyr::Gui
