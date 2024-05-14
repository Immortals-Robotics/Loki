#include "line.h"

namespace Tyr::Common
{
std::optional<Vec2> Line::intersect(const Line &t_line) const
{
    if (almostEqual(slope(), t_line.slope()))
    {
        return {};
    }
    else if (almostEqual(a, 0.0))
    {
        const float x = -c / b;
        return {{x, t_line.y(x)}};
    }
    else if (almostEqual(t_line.a, 0.0))
    {
        const float x = -t_line.c / t_line.b;
        return {{x, y(x)}};
    }
    else
    {
        const float x = (a * t_line.c - t_line.a * c) / (t_line.a * b - a * t_line.b);
        return {{x, y(x)}};
    }
}

std::vector<float> Line::abcFormula(const float t_a, const float t_b, const float t_c)
{
    // discriminant is b^2 - 4*a*c
    const float discr = t_b * t_b - 4 * t_a * t_c;

    if (std::abs(discr) < std::numeric_limits<float>::epsilon())
    {
        // discriminant = 0: only one solution
        return {-t_b / (2 * t_a)};
    }
    else if (discr < 0)
    {
        // discriminant < 0: no solutions
        return {};
    }
    else
    {
        // discriminant > 0: two solutions
        const float discr_sq = std::sqrt(discr);
        return {(-t_b + discr_sq) / (2 * t_a), (-t_b - discr_sq) / (2 * t_a)};
    }
}

// TODO: untested, and probably broken
std::vector<Vec2> Line::intersect(const Circle &t_circle) const
{
    // line:   x = -c/b (if a = 0)
    // circle: (x-t_circle.center.x)^2 + (y-t_circle.center.y)^2 = r^2,
    // fill in:(-c/b-t_circle.center.x)^2 + y^2 -2ky + t_circle.center.y^2 - r^2 = 0
    //         y^2 -2ky + (-c/b-t_circle.center.x)^2 + t_circle.center.y^2 - r^2 = 0
    // and determine solutions for y using abc-formula
    if (std::abs(a) < std::numeric_limits<float>::epsilon())
    {
        const auto sol = abcFormula(1, -2 * t_circle.center.y,
                                    ((-c / b) - t_circle.center.x) * ((-c / b) - t_circle.center.x) +
                                        t_circle.center.y * t_circle.center.y - t_circle.r * t_circle.r);

        return {{(-c / b), sol[0]}, {(-c / b), sol[1]}};
    }

    // ay + bx + c = 0 => y = -b/a x - c/a, with da = -b/a and db = -c/a
    // circle: (x-t_circle.center.x)^2 + (y-t_circle.center.y)^2 = r^2, with t_circle.center.x = center.x and
    // t_circle.center.y = center.y fill in:x^2 -2hx + t_circle.center.x^2 + (da*x-db)^2 -2k(da*x-db) +
    // t_circle.center.y^2 - r^2 = 0
    //         x^2 -2hx + t_circle.center.x^2 + da^2*x^2 + 2da*db*x + db^2 -2k*da*x -2k*db
    //                                                         + t_circle.center.y^2 - r^2 = 0
    //       (1+da^2)*x^2 + 2(da*db-t_circle.center.x-t_circle.center.y*da)*x + h2 + db^2  -2k*db + t_circle.center.y^2
    //       - r^2 = 0
    // and determine solutions for x using abc-formula
    // fill in x in original line equation to get y coordinate
    const float da = -b / a;
    const float db = -c / a;

    const float d_a = 1 + da * da;
    const float d_b = 2 * (da * db - t_circle.center.x - t_circle.center.y * da);
    const float d_c = t_circle.center.x * t_circle.center.x + db * db - 2 * t_circle.center.y * db +
                      t_circle.center.y * t_circle.center.y - t_circle.r * t_circle.r;

    const auto sol = abcFormula(d_a, d_b, d_c);

    return {{sol[0], da * sol[0] + db}, {sol[1], da * sol[1] + db}};
}
} // namespace Tyr::Common
