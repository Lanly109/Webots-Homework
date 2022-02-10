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

using namespace std;
using namespace webots;

const int lookDis = 20;
const double k = 1;
const double velocity = 40;
const double velocity2 = 30;

inline pair<int,int> real2pic(double X, double Y){ // 模拟现实左边转换成像素坐标
    return {int((3 - X) / 6 * 800), int((Y + 2.25) / 4.5 * 600)};
}

inline pair<double, double> pic2real(int x, int y){ // 像素坐标转换成模拟世界坐标
    return {3 - 6 * x * 1.0 / 800, -2.25 + 4.5 * y * 1.0 / 600};
}

class Path{
    vector<int> path; // 读取路径
    int l;
    int cols, rows;
    public:

    Path(){}
    Path(string s){
        FILE * fp = fopen(s.c_str(), "r");
        int tot = 0;
        fscanf(fp, "%d%d%d", &rows, &cols, &tot);
        while(tot--){
            int x;
            fscanf(fp, "%d", &x);
            path.push_back(x);
        }
    }

    inline int two2one(int x, int y){ // 将二维坐标转换成一维
        return x * cols + y;
    }

    inline pair<int,int> one2two(int id){ // 将一维坐标转换成二维
        return {id / cols, id % cols};
    }

    inline double calcDis(int x, int y){ // 计算距离
        return sqrt(x * x + y * y);
    }

    inline double dis(int x1, int y1, int x2, int y2){ // 计算两个二维点坐标距离
        return calcDis(x2 - x1, y2 - y1);
    }

    inline double dis(int u, int v){ // 计算两个一维点坐标距离
        auto [x1, y1] = one2two(u);
        auto [x2, y2] = one2two(v);
        return calcDis(x2 - x1, y2 - y1);
    }

    void clear(){
        curPoint = -1;
    }

    void setLookhead(int x){ // 设置lookahead distance
        l = x;
    }

    int curPoint = -1; 
 
    int findNearestPoint(int x, int y){ // 寻找距离当前位置最近的点
        int cc = curPoint;
        int cur = two2one(x, y);
        double ansdis = 1e9 + 7;
        curPoint = 0;
        for(size_t i = cc; i < path.size(); ++ i){
            double distance = dis(cur, path[i]);
            if (distance < ansdis){
                ansdis = distance;
                cc = i;
            }
        }
        // return one2two(path[curPoint]);
        return cc;
    }

    pair<int,int> findLookheadPoint(int x, int y){ // 寻找距离当前位置大于lookahead distance的最近的点
        int id = findNearestPoint(x, y);
        if (id > curPoint)
            curPoint = id;
        int cur = two2one(x, y);
        while(curPoint + 1 < path.size() && dis(cur, path[curPoint]) < l){
            ++ curPoint;
        }
        return one2two(path[curPoint]);
    }
};

int main()
{
    Path maze("./example.txt");
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
    { // 初始化电机和距离传感器
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

    //规定小车的朝向

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
        // 获取小车坐标以及速度

        double dismo[4];
        for(int i = 0; i < 4; ++ i){ // 获取距离传感器的值
            dismo[i] = dismotors[i]->getValue();
        }

        printf("Speed: %lf\n", v);
        // 根据速度设置Lookahead distance
        maze.setLookhead(min(30.0, v * k + lookDis));

        // 计算当前坐标在像素坐标系的坐标
        auto [x, y] = real2pic(Y, X);

        // 找到目标点
        auto [nx, ny] = maze.findLookheadPoint(x, y);

        // 换算成模拟世界坐标
        auto [nY, nX] = pic2real(nx, ny);

        // 计算距离
        double l = maze.dis(Y, X, nX, nY);

        // 计算以圆上行走的半径
        double R = l * l / 2 / min(abs(Y - nY), abs(X - nX));
        // 判断是否直线，直线则加速
        double vv = ((R == 0 || R > 2) ? 3 : 1);
        cout << vv << endl;

        // printf("%lf %lf %lf %lf\n", X, Y, nX, nY);
        // 判断当前应该向前向后以及向左向右
        int f = nX < X ? 1 : -1;
        int r = nY > Y ? 1 : -1;
        printf("%s %s\n", f == 1 ? "Forward" : "Back", r == 1 ? "Right" : "Left");
        for(int i = 0; i < 4; ++ i){ // 小车速度
            // speed1[i] = f * speed_forward[i] * fabs(nX - X);
            // speed2[i] = r * speed_rightward[i] * fabs(nY - Y);
            speed1[i] = f * speed_forward[i] * fabs(nX - X) * vv * min(1.0, (f == 1 ? dismo[0] : dismo[1]));
            speed2[i] = r * speed_rightward[i] * fabs(nY - Y) * vv * min(1.0, (r == 1 ? dismo[3] : dismo[2]));
        }

        // 判断当前小车朝向
        const double curx = robot->getSelf()->getOrientation()[0];
        const double cury = robot->getSelf()->getOrientation()[1];
        const double curz = robot->getSelf()->getOrientation()[2];
        printf("%lf %lf %lf\n", curx, cury, curz);
        printf("%lf %lf %lf\n", orix, oriy, oriz);
        // 如果偏离朝向，则根据偏离方向，设定自旋速度，并降低原先的行进速度
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
        for (int i = 0; i < 4; ++i) // 设置速度
        {
            motors[i]->setVelocity(speed1[i] + speed2[i] + speed3[i]);
        }
    }

    delete robot;
    return 0;
}
