#pragma once

namespace Tyr::Common
{
struct Color
{
    Color() = default;

    inline constexpr Color(const float t_r, const float t_g, const float t_b, const float t_a = 255.0f)
        : r(t_r), g(t_g), b(t_b), a(t_a)
    {}

    inline Color(const Protos::Immortals::Color &t_color) : Color(t_color.r(), t_color.g(), t_color.b(), t_color.a())
    {}

    inline void fillProto(Protos::Immortals::Color *const t_color) const
    {
        t_color->set_r(r);
        t_color->set_g(g);
        t_color->set_b(b);
        t_color->set_a(a);
    }

    [[nodiscard]] inline constexpr Color transparent() const
    {
        return Color{r, g, b, a / 4.0f};
    }

    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    float a = 255.0f;

    // Common colors that match raylib ones
    inline static constexpr Color lightGray()
    {
        return {200.0f, 200.0f, 200.0f, 255.0f};
    }

    inline static constexpr Color gray()
    {
        return {130.0f, 130.0f, 130.0f, 255.0f};
    }

    inline static constexpr Color darkGray()
    {
        return {80.0f, 80.0f, 80.0f, 255.0f};
    }

    inline static constexpr Color yellow()
    {
        return {253.0f, 249.0f, 0.0f, 255.0f};
    }

    inline static constexpr Color gold()
    {
        return {255.0f, 203.0f, 0.0f, 255.0f};
    }

    inline static constexpr Color orange()
    {
        return {255.0f, 161.0f, 0.0f, 255.0f};
    }

    inline static constexpr Color pink()
    {
        return {255.0f, 109.0f, 194.0f, 255.0f};
    }

    inline static constexpr Color red()
    {
        return {230.0f, 41.0f, 55.0f, 255.0f};
    }

    inline static constexpr Color maroon()
    {
        return {190.0f, 33.0f, 55.0f, 255.0f};
    }

    inline static constexpr Color green()
    {
        return {0.0f, 228.0f, 48.0f, 255.0f};
    }

    inline static constexpr Color lime()
    {
        return {0.0f, 158.0f, 47.0f, 255.0f};
    }

    inline static constexpr Color darkGreen()
    {
        return {0.0f, 117.0f, 44.0f, 255.0f};
    }

    inline static constexpr Color skyblue()
    {
        return {102.0f, 191.0f, 255.0f, 255.0f};
    }

    inline static constexpr Color blue()
    {
        return {0.0f, 121.0f, 241.0f, 255.0f};
    }

    inline static constexpr Color dark_blue()
    {
        return {0.0f, 82.0f, 172.0f, 255.0f};
    }

    inline static constexpr Color purple()
    {
        return {200.0f, 122.0f, 255.0f, 255.0f};
    }

    inline static constexpr Color violet()
    {
        return {135.0f, 60.0f, 190.0f, 255.0f};
    }

    inline static constexpr Color darkPurple()
    {
        return {112.0f, 31.0f, 126.0f, 255.0f};
    }

    inline static constexpr Color beige()
    {
        return {211.0f, 176.0f, 131.0f, 255.0f};
    }

    inline static constexpr Color brown()
    {
        return {127.0f, 106.0f, 79.0f, 255.0f};
    }

    inline static constexpr Color darkBrown()
    {
        return {76.0f, 63.0f, 47.0f, 255.0f};
    }

    inline static constexpr Color white()
    {
        return {255.0f, 255.0f, 255.0f, 255.0f};
    }

    inline static constexpr Color black()
    {
        return {0.0f, 0.0f, 0.0f, 255.0f};
    }

    inline static constexpr Color blank()
    {
        return {0.0f, 0.0f, 0.0f, 0};
    }

    inline static constexpr Color magenta()
    {
        return {255.0f, 0.0f, 255.0f, 255.0f};
    }
};
} // namespace Tyr::Common
