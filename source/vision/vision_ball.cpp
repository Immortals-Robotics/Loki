#include "vision.h"

namespace Tyr::Vision
{
void Vision::ProcessBalls()
{
    int balls_num = 0;

    // First we have to extract the balls!
    balls_num = ExtractBalls();

    // Now lets merge them!
    balls_num = MergeBalls(balls_num);

    // The most important part, The Kalman Filter!
    FilterBalls(balls_num);

    // We're almost done, only Prediction remains undone!
    predictBallForward();

    // Common::logDebug("ball pos: {}", m_state->ball.Position);
}

int Vision::ExtractBalls()
{
    int ans = 0;
    for (int i = 0; i < Common::Setting::kCamCount; i++)
    {
        if (Common::setting().use_camera[i])
        {
            for (int j = 0; j < frame[i].balls_size(); j++)
            {
                d_ball[ans] = frame[i].balls(j);
                ans++;
            }
        }
    }
    return ans;
}
int Vision::MergeBalls(int num)
{
    int balls_num = 0;
    for (int i = 0; i < num; i++)
    {
        const Common::Vec2 ball_i{d_ball[i].x(), d_ball[i].y()};

        for (int j = i + 1; j < num; j++)
        {
            const Common::Vec2 ball_j{d_ball[j].x(), d_ball[j].y()};

            if (ball_i.distanceTo(ball_j) < MERGE_DISTANCE)
            {
                const Common::Vec2 ball_merged = (ball_i + ball_j) / 2.0f;

                d_ball[i].set_x(ball_merged.x);
                d_ball[i].set_y(ball_merged.y);
                if (d_ball[i].has_z())
                    d_ball[i].set_z((d_ball[i].z() + d_ball[j].z()) / (float) 2.0);

                d_ball[j] = d_ball[num - 1];
                num--;

                j--;
            }
        }
        balls_num++;
    }

    return balls_num;
}

void Vision::FilterBalls(int num)
{
    int   id  = 100;
    float dis = std::numeric_limits<float>::max();

    const Common::Vec2 raw_ball{lastRawBall.x(), lastRawBall.y()};

    for (int i = 0; i < num; i++)
    {
        const Common::Vec2 ball_i{d_ball[i].x(), d_ball[i].y()};

        const float curr_dis = ball_i.distanceTo(raw_ball);
        if (curr_dis < dis)
        {
            dis = curr_dis;
            id  = i;
        }
    }

    if (dis < MAX_BALL_2FRAMES_DISTANCE)
    {
        float filtout[2][2];
        float filtpos[2] = {d_ball[id].x() / (float) 10.0, d_ball[id].y() / (float) 10.0};
        lastRawBall.CopyFrom(d_ball[id]);

        if (ball_not_seen > 0)
        {
            ball_kalman.initializePos(filtpos);
        }

        ball_kalman.updatePosition(filtpos, filtout);

        m_state->ball.Position.x = filtout[0][0];
        m_state->ball.Position.y = filtout[1][0];
        m_state->ball.velocity.x = filtout[0][1];
        m_state->ball.velocity.y = filtout[1][1];

        ball_not_seen         = 0;
        m_state->has_ball       = true;
        m_state->ball.seenState = Common::Seen;
    }

    else
    {
        ball_not_seen++;

        if (ball_not_seen > MAX_BALL_NOT_SEEN)
        {
            if (num > 0)
            {
                float filtout[2][2];
                float filtpos[2] = {d_ball[id].x() / (float) 10.0, d_ball[id].y() / (float) 10.0};
                lastRawBall.CopyFrom(d_ball[id]);
                ball_kalman.initializePos(filtpos);

                ball_kalman.updatePosition(filtpos, filtout);

                m_state->ball.Position.x = filtout[0][0];
                m_state->ball.Position.y = filtout[1][0];
                m_state->ball.velocity.x = filtout[0][1];
                m_state->ball.velocity.y = filtout[1][1];

                ball_not_seen         = 0;
                m_state->has_ball       = true;
                m_state->ball.seenState = Common::Seen;
            }
            else
            {
                m_state->ball.velocity.x = 0;
                m_state->ball.velocity.y = 0;

                m_state->ball.Position.x /= (float) 10.0;
                m_state->ball.Position.y /= (float) 10.0;

                lastRawBall.set_x(0.0f);
                lastRawBall.set_y(0.0f);

                m_state->has_ball       = false;
                m_state->ball.seenState = Common::CompletelyOut;
            }
        }
        else
        {
            m_state->ball.velocity.x /= (float) 10.0;
            m_state->ball.velocity.y /= (float) 10.0;

            m_state->ball.Position.x /= (float) 10.0;
            m_state->ball.Position.y /= (float) 10.0;

            m_state->ball.seenState = Common::TemprolilyOut;
        }
    }
}

void Vision::predictBallForward()
{
    m_state->ball.Position.x /= (float) 100.0;
    m_state->ball.Position.y /= (float) 100.0;
    m_state->ball.velocity.x /= (float) 100.0;
    m_state->ball.velocity.y /= (float) 100.0;
    float k          = 0.25f; // velocity derate every sec(units (m/s)/s)
    float frame_rate = 61.0f;
    float tsample    = (float) 1.0f / (float) frame_rate;

    float vx_vision = m_state->ball.velocity.x;
    float vy_vision = m_state->ball.velocity.y;

    float xpos_vision = m_state->ball.Position.x;
    float ypos_vision = m_state->ball.Position.y;

    float vball_vision = float(sqrt(vx_vision * vx_vision + vy_vision * vy_vision));

    float t;
    if (m_state->ball.seenState == Common::TemprolilyOut)
        t = tsample;
    else
        t = PREDICT_STEPS * tsample;

    float v     = vball_vision - k * t;
    float dist0 = vball_vision * t - k * (t * t) / 2.0f;

    float dist;
    float vball_pred;

    // if speed turns out to be negative..it means that ball has stopped, so calculate that amount of
    // distance traveled
    if (v < 0)
    {
        vball_pred = 0.0f;
        dist       = (vball_vision * vball_vision) * k / 2.0f;
        // i.e the ball has stopped, so take a newer vision data for the prediction
    }
    else
    {
        vball_pred = v;
        dist       = dist0;
    }

    if (vball_vision != 0)
    {
        m_state->ball.velocity.x = vball_pred * (vx_vision) / vball_vision;
        m_state->ball.velocity.y = vball_pred * (vy_vision) / vball_vision;
        m_state->ball.Position.x = (xpos_vision + dist * (vx_vision) / vball_vision);
        m_state->ball.Position.y = (ypos_vision + dist * (vy_vision) / vball_vision);
    }

    m_state->ball.velocity.x *= (float) 1000.0;
    m_state->ball.velocity.y *= (float) 1000.0;
    m_state->ball.Position.x *= (float) 1000.0;
    m_state->ball.Position.y *= (float) 1000.0;
}

} // namespace Tyr::Vision