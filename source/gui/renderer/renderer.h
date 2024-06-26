#pragma once
#include "../menu/filter.h"

namespace Tyr::Gui
{
class Renderer
{
public:
    Renderer();

    void begin(const Common::FieldState &t_field, Common::Rect t_window_rect);
    void end();

    inline void draw(const Common::Vec2 &t_point, const Common::Color &t_color, float t_thickness = 10.0f) const;
    inline void draw(const Common::Rect &t_rect, const Common::Color &t_color, bool t_is_filled,
                     float t_thickness = 10.0f) const;
    inline void draw(const Common::Circle &t_circle, const Common::Color &t_color, bool t_is_filled,
                     float t_thickness = 10.0f) const;
    inline void drawCircleSector(const Common::Circle &t_circle, const Common::Color &t_color, float t_start_angle,
                                 float t_end_angle, bool t_is_filled) const;
    inline void draw(const Common::Line &t_line, const Common::Color &t_color, float t_thickness = 10.0f) const;
    inline void draw(const Common::LineSegment &t_line, const Common::Color &t_color, float t_thickness = 10.0f) const;
    inline void draw(const Common::Triangle &t_triangle, const Common::Color &t_color, bool t_is_filled = true,
                     float t_thickness = 10.0f) const;

    inline void draw(const Common::Vec2 &t_pos, const std::string &t_text, const Common::Color &t_color,
                     float t_font_size = 200) const;

    void draw(const Common::RawRobotState &t_robot);
    void draw(const Common::RawBallState &t_ball);
    void draw(const Common::RawWorldState &t_world);

    void draw(const Common::RobotState &t_robot);
    void draw(const Common::BallState &t_ball);
    void draw(const Common::WorldState &t_world);

    void draw(const Common::FieldState &t_field);

    void draw(const Common::Referee::State &t_referee_state, const Common::FieldState &t_field);

    void draw(const Common::Debug::Wrapper &t_wrapper, const FilterMenu::Map &t_map) const;

    Common::Vec2 mousePosition() const
    {
        return Common::Vec2{mirrorY(GetScreenToWorld2D(GetMousePosition(), m_camera))};
    }

private:
    static constexpr float kRobotArcAngle = 50.0f;
    static constexpr float kWindowBorder  = 8.0f;

    static Vector2 mirrorY(const Vector2 t_v)
    {
        return {.x = t_v.x, .y = -t_v.y};
    }

    static Rectangle mirrorY(const Rectangle t_rect)
    {
        return {.x = t_rect.x, .y = -(t_rect.y + t_rect.height), .width = t_rect.width, .height = t_rect.height};
    }

    Camera2D m_camera;
    Font     m_font;

    std::optional<Common::Vec2> m_clicked_pos;
};
} // namespace Tyr::Gui

#include "draw_inl.h"
