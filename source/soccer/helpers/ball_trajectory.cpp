#include "ball_trajectory.h"

namespace Tyr::Soccer
{
void BallTrajectory::update(const Common::BallState &t_ball)
{
    if (m_empty)
    {
        m_ball_hist.fill(t_ball.position);
        m_empty = false;
    }
    else
    {
        std::shift_left(m_ball_hist.begin(), m_ball_hist.end(), 1);
        m_ball_hist.back() = t_ball.position;
    }

    if (t_ball.velocity.length() > 0.0f)
    {
        if (m_ball.velocity.x == 0)
            m_vision_dir.add(100.0f);
        else
            m_vision_dir.add(std::fabs(m_ball.velocity.y / m_ball.velocity.x));
    }

    m_ball = t_ball;
}

void BallTrajectory::calculate() const
{
    // TODO: replace with epsilon check
    if (m_ball.velocity.length() == 0.0f)
        return;

    bool isVertical = m_vision_dir.current() > 1.0f;

    std::vector<float>             ball_x;
    std::vector<float>             ball_y;
    Common::MedianFilter<float, 5> dMedian;

    Common::Linear m_ball_line;

    int i;
    for (i = m_ball_hist.size() - 1; i >= 0; i--)
    {
        if (ball_x.size() > 1)
        {

            float d;
            if (isVertical)
            {
                m_ball_line = Common::Linear::calculate(ball_x.size(), &ball_y[0], &ball_x[0]);
                d           = m_ball_line.getDisToPoint(m_ball_hist[i].yx());
            }
            else
            {
                m_ball_line = Common::Linear::calculate(ball_x.size(), &ball_x[0], &ball_y[0]);
                d           = m_ball_line.getDisToPoint(m_ball_hist[i]);
            }

            dMedian.add(d);
            if ((dMedian.current() > 50) && (m_ball_hist.size() - i > 5))
            {
                break;
            }
        }
        ball_x.push_back(m_ball_hist[i].x);
        ball_y.push_back(m_ball_hist[i].y);
    }

    i++;

    if (isVertical)
        m_ball_line.chapeKon();


    if (m_ball_line.isAmoodi())
    {
        Common::debug().draw(Common::LineSegment{Common::Vec2(m_ball_line.getXIntercept(), -2000),
                                                 Common::Vec2(m_ball_line.getXIntercept(), 2000)},
                             Common::Color::purple());
    }
    Common::debug().draw(Common::LineSegment{Common::Vec2(m_ball.position.x, m_ball_line.getValue(m_ball.position.x)),
                                             Common::Vec2(m_ball_hist[i].x, m_ball_line.getValue(m_ball_hist[i].x))});

    if (!m_ball_line.isAmoodi())
    {
        Common::Line new_line(1.0, -m_ball_line.getSlope(), -m_ball_line.getIntercept());
        Common::Vec2 ballN, lastN;
        ballN = new_line.closestPoint(m_ball.position);
        lastN = new_line.closestPoint(m_ball_hist[i]);
        Common::debug().draw(Common::LineSegment{ballN, lastN},
                             isVertical ? Common::Color::yellow() : Common::Color::red());
    }

    for (i = 0; i < m_ball_hist.size(); i += 1)
    {
        Common::debug().draw(Common::Circle{m_ball_hist[i], 5.0f + 30.0f * (static_cast<float>(i) / static_cast<float>(m_ball_hist.size()))},
                             Common::Color::red().transparent());
    }

    Common::debug().draw(Common::LineSegment{m_ball_hist.front(), m_ball_hist.back()},
                         Common::Color::blue().transparent());
}
} // namespace Tyr::Soccer
