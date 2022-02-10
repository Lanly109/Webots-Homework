#include <webots/Robot.hpp>
#include <webots/Display.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/GPS.hpp>

#include <iostream>
#include <algorithm>
#include <limits>
#include <string>

using namespace std;
using namespace webots;

const int lookDis = 20;
const double k = 1;
const double velocity = 10;
const double velocity2 = 10;

int main()
{

    // Robot *robot = new Robot();
    Supervisor *robot = new Supervisor();
    int timeStep = (int)robot->getBasicTimeStep();
    cout << timeStep << endl;

    Keyboard keyboard;
    keyboard.enable(1);

    GPS* gps = robot->getGPS("gps");
    gps->enable(1);

    Motor *motors[4];

    char wheelsNames[4][8] = {"motor1", "motor2", "motor3", "motor4"};
	double speed1[4];
	double speed2[4];
	double speed3[4];


    for (int i = 0; i < 4; i++)
    {
        motors[i] = robot->getMotor(wheelsNames[i]);
        motors[i]->setPosition(std::numeric_limits<double>::infinity());
        motors[i]->setVelocity(0.0);
        speed1[i] = 0;
        speed2[i] = 0;
        speed3[i] = 0;
    }

	double speed_forward[4] = { velocity ,velocity ,velocity ,velocity };
	double speed_backward[4] = { -velocity ,-velocity ,-velocity ,-velocity };
	double speed_leftward[4] = { velocity ,-velocity ,velocity ,-velocity };
	double speed_rightward[4] = { -velocity ,velocity ,-velocity ,velocity };

	double speed_leftCircle[4] = { velocity2 ,-velocity2 ,-velocity2 ,velocity2 };
	double speed_rightCircle[4] = { -velocity2 ,velocity2 ,velocity2 ,-velocity2 };

    for(int i = 0; i < 4; ++ i)
        speed1[i] = speed_forward[i];

    while (robot->step(timeStep) != -1)
    {

        int keyValue = 0;
        keyValue = keyboard.getKey();
        if (keyValue == 'W')
        {
            for (int i = 0; i < 4; ++i)
            {
                speed1[i] = speed_forward[i];
            }
        }
        else if (keyValue == 'S')
        {
            for (int i = 0; i < 4; ++i)
            {
                speed1[i] = speed_backward[i];
            }
        }
        else if (keyValue == 'A')
        {
            for (int i = 0; i < 4; ++i)
            {
                speed1[i] = speed_leftward[i];
            }
        }
        else if (keyValue == 'D')
        {
            for (int i = 0; i < 4; ++i)
            {
                speed1[i] = speed_rightward[i];
            }
        }
        else if (keyValue == 'Q'){
            for (int i = 0; i < 4; ++ i)
                speed1[i] = speed_leftCircle[i];
        }else if (keyValue == 'E'){
            for (int i = 0; i < 4; ++ i)
                speed1[i] = speed_rightCircle[i];
        }else
        {
            for (int i = 0; i < 4; ++i)
            {
                speed1[i] = 0;
            }
        }

        for (int i = 0; i < 4; ++i)
        {
            motors[i]->setVelocity(speed1[i] + speed2[i] + speed3[i]);
        }
    }

    delete robot;
    return 0;
}
