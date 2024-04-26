#include "vision.h"

namespace Loki::Vision
{
// Don't add prediction to Ball or Opponents if both velocities are below this threshold
const float IGNORE_PREDICTION = 0.045f;

// If the filtering process yields velocities above these values, reset the filter state
// All these are in metres/sec
const float BALL_ERROR_VELOCITY_SQUARED     = 1960000.0f;
const float OPPONENT_ERROR_VELOCITY_SQUARED = 200000.0f;

std::ofstream outfile("outf.txt");
std::ofstream delay_data("delay.txt");
unsigned int  fr_num = 0;

void VisionModule::ProcessRobots(Common::WorldState *state)
{
    int robots_num = 0;

    // Blue Robots
    // First we have to extract the robots!
    robots_num = ExtractBlueRobots();

    // Now lets merge them!
    robots_num = MergeRobots(robots_num);

    // The most important part, The Kalman Filter!
    FilterRobots(robots_num, this->our_color);

    // Yellow Robots
    // First we have to extract the robots!
    robots_num = ExtractYellowRobots();

    // Now lets merge them!
    robots_num = MergeRobots(robots_num);

    // The most important part, The Kalman Filter!
    FilterRobots(robots_num, !(this->our_color));

    // We're almost done, only Prediction remains undone!
    predictRobotsForward(state);
    // RunANN(state);
    // PredictWithANN(state);
    // TrainANN(0.1f);
    // RunANN(state);

    // Now we send Robots States to the AI!
    SendStates(state);

    // int cmdIndex = state->lastCMDS[6][10].X;
    // delay_data << fr_num++ << "	" << state->OwnRobot[6].Position.X << "	" << state->OwnRobot[6].Position.Y << "	" <<
    // state->lastCMDS[6][cmdIndex].X << "	" << state->lastCMDS[6][cmdIndex].Y << std::endl;
}
int VisionModule::ExtractBlueRobots(void)
{
    int ans = 0;
    for (int i = 0; i < Common::Setting::kCamCount; i++)
    {
        if (Common::setting().use_camera[i])
        {
            for (int j = 0; j < std::min((int) Common::Setting::kMaxRobots, frame[i].robots_blue_size()); j++)
            {
                robot[ans] = frame[i].robots_blue(j);
                ans++;
            }
        }
    }
    return ans;
}

int VisionModule::ExtractYellowRobots(void)
{
    int ans = 0;
    for (int i = 0; i < Common::Setting::kCamCount; i++)
    {
        if (Common::setting().use_camera[i])
        {
            for (int j = 0; j < std::min((int) Common::Setting::kMaxRobots, frame[i].robots_yellow_size()); j++)
            {
                robot[ans] = frame[i].robots_yellow(j);
                ans++;
            }
        }
    }
    return ans;
}

int VisionModule::MergeRobots(int num)
{
    int robots_num = 0;
    for (int i = 0; i < num; i++)
    {
        for (int j = i + 1; j < num; j++)
        {
            if (POWED_DIS(robot[i].x(), robot[i].y(), robot[j].x(), robot[j].y()) < MERGE_DISTANCE)
            {
                robot[i].set_x((robot[i].x() + robot[j].x()) / (float) 2.0);
                robot[i].set_y((robot[i].y() + robot[j].y()) / (float) 2.0);
                robot[j] = robot[num - 1];
                num--;

                j--;
            }
        }
        robots_num++;
    }

    return robots_num;
}

void VisionModule::FilterRobots(int num, bool own)
{
    float filtout[2][2];
    float filtpos[2];

    for (int i = 0; i < Common::Setting::kMaxRobots; i++)
    {
        bool found = false;
        for (int j = 0; j < num; j++)
        {
            if (robot[j].robot_id() == i)
            {
                found = true;

                filtpos[0] = robot[j].x() / (float) 10.0;
                filtpos[1] = robot[j].y() / (float) 10.0;

                if (robot_not_seen[own][i] > 0)
                {
                    robot_kalman[own][i].initializePos(filtpos);
                    AngleFilter[own][i].reset();
                }

                robot_not_seen[own][i] = 0;

                robot_kalman[own][i].updatePosition(filtpos, filtout);

                // if ( robot[j].robot_id() == 5 )
                // robot[j].set_orientation ( robot[j].orientation() + 0.135 );

                AngleFilter[own][i].AddData((robot[j].orientation() - rawAngles[own][i]) * 61.0f);
                rawAngles[own][i] = robot[j].orientation();
                // if ( fabs ( (AngleFilter[own][i].GetCurrent()*180.0f/3.1415f) - robotState[own][i].AngularVelocity )
                // > 30.0f ) 	AngleFilter[own][i].reset(); else
                {
                    robotState[own][i].AngularVelocity = AngleFilter[own][i].GetCurrent();
                    robotState[own][i].AngularVelocity *= 180.0f / 3.1415f;
                }
                robotState[own][i].Angle = robot[j].orientation() * 180.0f / 3.1415f;
                // if ( robot[j].robot_id() == 1 )
                //	std::cout << robotState[own][i].AngularVelocity << std::endl;

                // Make sure our filtered velocities are reasonable
                if (fabs(robotState[own][i].AngularVelocity) < 20.0f)
                    robotState[own][i].AngularVelocity = 0.0f;
                /*if ( fabs ( robotState[own][i].AngularVelocity ) > 4000.0f )
                 robotState[own][i].AngularVelocity = 0.0f;*/
            }
        }

        if (!found)
        {
            robot_not_seen[own][i]++;
            if (robot_not_seen[own][i] >= MAX_ROBOT_NOT_SEEN + 1)
                robot_not_seen[own][i] = MAX_ROBOT_NOT_SEEN + 1;

            // robotState[own][i].Angle = 0.0f;
            robotState[own][i].AngularVelocity = 0.0f;

            // robotState[own][i].velocity.x = 0.0f;
            // robotState[own][i].velocity.y = 0.0f;

            // robotState[own][i].velocity.direction = 0.0f;
            // robotState[own][i].velocity.magnitude = 0.0f;

            // robotState[own][i].Position.X += robotState[own][i].velocity.x / 61.0;
            // robotState[own][i].Position.Y += robotState[own][i].velocity.y / 61.0;

            //}
        }

        else
        {
            robotState[own][i].Position.x = filtout[0][0];
            robotState[own][i].Position.y = filtout[1][0];
            robotState[own][i].velocity.x = filtout[0][1];
            robotState[own][i].velocity.y = filtout[1][1];

            // Make sure our filtered velocities are reasonable
            if ((robotState[own][i].velocity.x * robotState[own][i].velocity.x) +
                    (robotState[own][i].velocity.y * robotState[own][i].velocity.y) >
                OPPONENT_ERROR_VELOCITY_SQUARED)
            {
                robotState[own][i].velocity.x = 0.0f;
                robotState[own][i].velocity.y = 0.0f;
            }

            if (fabs(robotState[own][i].velocity.x) < IGNORE_PREDICTION * 2.0f)
                robotState[own][i].velocity.x = 0.0f;
            if (fabs(robotState[own][i].velocity.y) < IGNORE_PREDICTION * 2.0f)
                robotState[own][i].velocity.y = 0.0f;

            robotState[own][i].Position.x *= (float) (10.0);
            robotState[own][i].Position.y *= (float) (10.0);
            robotState[own][i].velocity.x *= (float) (10.0);
            robotState[own][i].velocity.y *= (float) (10.0);

            robotState[own][i].velocity.direction =
                atan((robotState[own][i].velocity.y) / (robotState[own][i].velocity.x));
            robotState[own][i].velocity.direction *= 180.0f / 3.1415f;
            if (robotState[own][i].velocity.x < 0)
                robotState[own][i].velocity.direction += 180;
            while (robotState[own][i].velocity.direction > 180)
                robotState[own][i].velocity.direction -= 360;
            while (robotState[own][i].velocity.direction < -180)
                robotState[own][i].velocity.direction += 360;

            robotState[own][i].velocity.magnitude =
                sqrt((robotState[own][i].velocity.x * robotState[own][i].velocity.x) +
                     (robotState[own][i].velocity.y * robotState[own][i].velocity.y));
        }
    }
}

void VisionModule::predictRobotsForward(Common::WorldState *state)
{
    for (int own = 1; own < 2; own++)
    {
        for (int i = 0; i < Common::Setting::kMaxRobots; i++)
        {
            /*robotState[own][i].Position.X /= (float)(1000.0);
             robotState[own][i].Position.Y /= (float)(1000.0);
             robotState[own][i].velocity.x /= (float)(1000.0);
             robotState[own][i].velocity.y /= (float)(1000.0);*/

            //====================================
            //====================================
            //====================================
            // Predict the robot to go forward
            /*if ( ( i == 0 ) || ( i == 4 ) )
            {
                robotState[own][i].Position.X = robotState[own][i].Position.X + robotState[own][i].velocity.x / (
            PREDICT_STEPS * 2.0f );

                // Predict the robot to go forward
                robotState[own][i].Position.Y = robotState[own][i].Position.Y + robotState[own][i].velocity.y / (
            PREDICT_STEPS * 2.0f );

                // Predict the robot to go forward
                //robotState[own][i].Angle = robotState[own][i].Angle + robotState[own][i].AngularVelocity / (
            PREDICT_STEPS * 4.0f );
            }*/

            //====================================
            //====================================
            //====================================
            /*robotState[own][i].Position.X *= (float)(1000.0);
             robotState[own][i].Position.Y *= (float)(1000.0);
             robotState[own][i].velocity.x *= (float)(1000.0);
             robotState[own][i].velocity.y *= (float)(1000.0);*/

            robotState[own][i].Position.x =
                robotState[own][i].Position.x + robotState[own][i].velocity.x / (PREDICT_STEPS * 2.0f);

            // Predict the robot to go forward
            robotState[own][i].Position.y =
                robotState[own][i].Position.y + robotState[own][i].velocity.y / (PREDICT_STEPS * 2.0f);

            // Predict the robot to go forward
            robotState[own][i].Angle =
                robotState[own][i].Angle + robotState[own][i].AngularVelocity / (PREDICT_STEPS * 4.0f);
        }
    }

    for (int i = 0; i < Common::Setting::kMaxRobots; i++)
    {
        if (robotState[0][i].seenState != Common::Seen)
        {
            robotState[0][i].Position.x =
                robotState[0][i].Position.x + state->lastCMDS[i][(int) state->lastCMDS[i][10].x].x / 1.2f;
            robotState[0][i].Position.y =
                robotState[0][i].Position.y + state->lastCMDS[i][(int) state->lastCMDS[i][10].x].y / 1.2f;
        }
        else
        {
            for (int j = 0; j < 10; j++)
            {
                robotState[0][i].Position.x = robotState[0][i].Position.x + state->lastCMDS[i][j].x / 1.4f;
                robotState[0][i].Position.y = robotState[0][i].Position.y + state->lastCMDS[i][j].y / 1.4f;
                // if (( i == 3 ) )
                //	robotState[0][i].Angle = robotState[0][i].Angle - state -> lastCMDS[i][j].Z * 0.04f;
            }
        }
        if (robotState[0][i].Angle > 180)
            robotState[0][i].Angle -= 360.0f;
        if (robotState[0][i].Angle < -180)
            robotState[0][i].Angle += 360.0f;
    }
    // outfile << robotState[0][1].AngularVelocity << std::endl;
}

void VisionModule::SendStates(Common::WorldState *state)
{
    state->ownRobots_num = 0;
    for (int i = 0; i < Common::Setting::kMaxRobots; i++)
    {
        robotState[0][i].vision_id = i;

        state->ownRobots_num++;
        if (robot_not_seen[0][i] == 0)
        {
            robotState[0][i].seenState = Common::Seen;
        }
        else if (robot_not_seen[0][i] < MAX_ROBOT_NOT_SEEN)
        {
            robotState[0][i].seenState = Common::TemprolilyOut;
        }
        else
        {
            robotState[0][i].seenState = Common::CompletelyOut;
            state->ownRobots_num--;
        }

        if (robot_not_seen[0][i] < MAX_ROBOT_SUBSITUTE)
        {
            robotState[0][i].OutForSubsitute = false;
        }
        else
        {
            robotState[0][i].OutForSubsitute = true;
        }

        state->OwnRobot[i] = robotState[0][i];
    }

    state->oppRobots_num = 0;
    for (int i = 0; i < Common::Setting::kMaxRobots; i++)
    {
        robotState[1][i].vision_id = i;

        state->oppRobots_num++;
        if (robot_not_seen[1][i] == 0)
        {
            robotState[1][i].seenState = Common::Seen;
        }
        else if (robot_not_seen[1][i] < MAX_ROBOT_NOT_SEEN)
        {
            robotState[1][i].seenState = Common::TemprolilyOut;
        }
        else
        {
            robotState[1][i].seenState = Common::CompletelyOut;
            state->oppRobots_num--;
        }

        state->OppRobot[i] = robotState[1][i];
    }
}
} // namespace Loki::Vision