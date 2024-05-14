#pragma once

#include "helpers.h"

namespace Tyr::Common
{
struct Angle;

struct Vec3
{
    Vec3() = default;
    constexpr Vec3(float t_f) : Vec3(t_f, t_f, t_f)
    {}
    constexpr Vec3(float t_x, float t_y, float t_z) : x(t_x), y(t_y), z(t_z)
    {}

    Vec3(const Protos::Immortals::Vec3 &t_v) : Vec3(t_v.x(), t_v.y(), t_v.z())
    {}

    void fillProto(Protos::Immortals::Vec3 *const t_v) const
    {
        t_v->set_x(x);
        t_v->set_y(y);
        t_v->set_z(z);
    }

    [[nodiscard]] inline Vec3 normalized() const
    {
        const float length = this->length();

        return length == 0.0 ? Vec3{0.0, 0.0, 0.0} : *this / length;
    }

    [[nodiscard]] inline float length() const
    {
        return sqrt(x * x + y * y + z * z);
    }

    [[nodiscard]] inline float dot(Vec3 t_v)
    {
        return x * t_v.x + y * t_v.y + z * t_v.z;
    }

    [[nodiscard]] inline Vec3 operator+(Vec3 t_v) const
    {
        return Vec3{x + t_v.x, y + t_v.y, z + t_v.z};
    }

    [[nodiscard]] inline Vec3 operator-(Vec3 t_v) const
    {
        return Vec3{x - t_v.x, y - t_v.y, z - t_v.z};
    }

    [[nodiscard]] inline Vec3 operator*(Vec3 t_v) const
    {
        return Vec3{x * t_v.x, y * t_v.y, z * t_v.z};
    }

    [[nodiscard]] inline Vec3 operator/(Vec3 t_v) const
    {
        return Vec3{x / t_v.x, y / t_v.y, z / t_v.z};
    }

    [[nodiscard]] inline Vec3 operator*(float t_d) const
    {
        return Vec3{x * t_d, y * t_d, z * t_d};
    }

    [[nodiscard]] inline Vec3 operator/(float t_d) const
    {
        return Vec3{x / t_d, y / t_d, z / t_d};
    }

    inline Vec3 &operator+=(Vec3 t_v)
    {
        x += t_v.x;
        y += t_v.y;
        z += t_v.z;

        return *this;
    }

    inline Vec3 &operator-=(Vec3 t_v)
    {
        x -= t_v.x;
        y -= t_v.y;
        z -= t_v.z;

        return *this;
    }

    inline Vec3 &operator*=(Vec3 t_v)
    {
        x *= t_v.x;
        y *= t_v.y;
        z *= t_v.z;

        return *this;
    }

    inline Vec3 &operator/=(Vec3 t_v)
    {
        x /= t_v.x;
        y /= t_v.y;
        z /= t_v.z;

        return *this;
    }

    inline Vec3 &operator*=(float t_d)
    {
        x *= t_d;
        y *= t_d;
        z *= t_d;

        return *this;
    }

    inline Vec3 &operator/=(float t_d)
    {
        x /= t_d;
        y /= t_d;
        z /= t_d;

        return *this;
    }

    [[nodiscard]] inline Vec3 operator-() const
    {
        return Vec3{-x, -y, -z};
    }

    [[nodiscard]] inline Vec3 operator+() const
    {
        return *this;
    }

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

} // namespace Tyr::Common

template <>
struct fmt::formatter<Tyr::Common::Vec3> : fmt::formatter<std::string>
{
    auto format(Tyr::Common::Vec3 t_v, format_context &t_ctx) const
    {
        return fmt::format_to(t_ctx.out(), "[{}, {}, {}]", t_v.x, t_v.y, t_v.z);
    }
};