#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/GPS.hpp>

#include <iostream>
#include <algorithm>
#include <limits>
#include <string>
#include "tu.h"

using namespace std;
using namespace webots;

const int lookDis = 20;
const double k = 1;
const double velocity = 40;
const double velocity2 = 30;

inline pair<int,int> real2pic(double X, double Y){
    return {int((3 - X) / 6 * 800), int((Y + 2.25) / 4.5 * 600)};
}

inline pair<double, double> pic2real(int x, int y){
    return {3 - 6 * x * 1.0 / 800, -2.25 + 4.5 * y * 1.0 / 600};
}

int main()
{
    tuP maze("./maze.txt");
    maze.setLookhead(lookDis);

    // Robot *robot = new Robot();
    Supervisor *robot = new Supervisor();
    int timeStep = (int)robot->getBasicTimeStep();
    cout << timeStep << endl;

    Keyboard keyboard;
    keyboard.enable(1);

    GPS* gps = robot->getGPS("gps");
    gps->enable(1);

    Motor *motors[4];
    DistanceSensor *dismotors[4];

    char wheelsNames[4][8] = {"motor1", "motor2", "motor3", "motor4"};
    char distanceNames[4][8] = {"front", "back", "left", "right"};
	double speed1[4];
	double speed2[4];
	double speed3[4];


    for (int i = 0; i < 4; i++)
    {
        motors[i] = robot->getMotor(wheelsNames[i]);
        dismotors[i] = robot->getDistanceSensor(distanceNames[i]);
        dismotors[i]->enable(1);
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

    double orix = -1;
    double oriy = 0;
    double oriz = 0;

    while (robot->step(timeStep) != -1)
    {
        // const double* rotation = field->getSFRotation();
        // cout << field->getSFBool() << endl;

        double X = gps->getValues()[0];
        double Y = gps->getValues()[1];
        double Z = gps->getValues()[2];
        double v = gps->getSpeed();

        double dismo[4];
        for(int i = 0; i < 4; ++ i){
            dismo[i] = dismotors[i]->getValue();
        }

        cout << dismo[0] << ' ' << dismo[1] << ' ' << dismo[2] << ' ' << dismo[3] << endl;

        printf("Speed: %lf\n", v);
        maze.setLookhead(min(30.0, v * k + lookDis));

        auto [x, y] = real2pic(Y, X);

        auto [nx, ny] = maze.findLookheadPoint(x, y);

        auto [nY, nX] = pic2real(nx, ny);

        double l = maze.dis(Y, X, nX, nY);

        double R = l * l / 2 / abs(X - nX);
        double vv = (R > 2 ? 3 : 1);


        // printf("%lf %lf %lf %lf\n", X, Y, nX, nY);
        int f = nX < X ? 1 : -1;
        int r = nY > Y ? 1 : -1;
        printf("%s %s\n", f == 1 ? "Forward" : "Back", r == 1 ? "Right" : "Left");
        for(int i = 0; i < 4; ++ i){
            // speed1[i] = f * speed_forward[i] * fabs(nX - X);
            // speed2[i] = r * speed_rightward[i] * fabs(nY - Y);
            speed1[i] = f * speed_forward[i] * fabs(nX - X) * vv * min(1.0, (f == 1 ? dismo[0] : dismo[1]));
            speed2[i] = r * speed_rightward[i] * fabs(nY - Y) * vv * min(1.0, (r == 1 ? dismo[3] : dismo[2]));
        }

        const double curx = robot->getSelf()->getOrientation()[0];
        const double cury = robot->getSelf()->getOrientation()[1];
        const double curz = robot->getSelf()->getOrientation()[2];
        printf("%lf %lf %lf\n", curx, cury, curz);
        printf("%lf %lf %lf\n", orix, oriy, oriz);
        if (abs(curx - orix) > 0.1){
            if (cury < 0){
                printf("LEFT rec\n");
                for(int i = 0; i < 4; ++ i){
                    speed1[i] *= 0.01;
                    speed2[i] *= 0.01;
                    speed3[i] = speed_leftCircle[i] * abs(curx - orix);
                }
            }else{
                printf("RIGHT rec\n");
                for(int i = 0; i < 4; ++ i){
                    speed1[i] *= 0.01;
                    speed2[i] *= 0.01;
                    speed3[i] = speed_rightCircle[i] * abs(curx - orix);
                }
            }
        }else{
            for(int i = 0; i < 4; ++ i)
                speed3[i] = 0;
        }

        // int keyValue = 0;
        // keyValue = keyboard.getKey();
        // if (keyValue == 'W')
        // {
        //     for (int i = 0; i < 4; ++i)
        //     {
        //         speed1[i] = speed_forward[i];
        //     }
        // }
        // else if (keyValue == 'S')
        // {
        //     for (int i = 0; i < 4; ++i)
        //     {
        //         speed1[i] = speed_backward[i];
        //     }
        // }
        // else if (keyValue == 'A')
        // {
        //     for (int i = 0; i < 4; ++i)
        //     {
        //         speed1[i] = speed_leftward[i];
        //     }
        // }
        // else if (keyValue == 'D')
        // {
        //     for (int i = 0; i < 4; ++i)
        //     {
        //         speed1[i] = speed_rightward[i];
        //     }
        // }
        // else if (keyValue == 'Q'){
        //     for (int i = 0; i < 4; ++ i)
        //         speed1[i] = speed_leftCircle[i];
        // }else if (keyValue == 'E'){
        //     for (int i = 0; i < 4; ++ i)
        //         speed1[i] = speed_rightCircle[i];
        // }else
        // {
        //     for (int i = 0; i < 4; ++i)
        //     {
        //         speed1[i] = 0;
        //     }
        // }
        //
        for (int i = 0; i < 4; ++i)
        {
            motors[i]->setVelocity(speed1[i] + speed2[i] + speed3[i]);
        }
    }

    delete robot;
    return 0;
}
