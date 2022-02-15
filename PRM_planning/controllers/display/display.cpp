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

#define RED 0xBB2222
#define GREEN 0x22BB11
#define BLUE 0x2222BB

const int lookDis = 20;

inline pair<int,int> real2pic(double X, double Y){
    return {int((3 - X) / 6 * 800), int((Y + 2.25) / 4.5 * 600)};
}

inline pair<double, double> pic2real(int x, int y, int w, int h){ // 获取像素坐标系在演示坐标系下的坐标
    return {h * (1.0 * x / 800), w * (1.0 * y / 600)};
}

class Path{
    vector<int> path;
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

    inline int two2one(int x, int y){
        return x * cols + y;
    }

    inline pair<int,int> one2two(int id){
        return {id / cols, id % cols};
    }

    inline double calcDis(int x, int y){
        return sqrt(x * x + y * y);
    }

    inline double dis(int x1, int y1, int x2, int y2){
        return calcDis(x2 - x1, y2 - y1);
    }

    inline double dis(int u, int v){
        auto [x1, y1] = one2two(u);
        auto [x2, y2] = one2two(v);
        return calcDis(x2 - x1, y2 - y1);
    }

    void clear(){
        curPoint = -1;
    }

    void setLookhead(int x){
        l = x;
    }

    int curPoint = -1; 

    int findNearestPoint(int x, int y){
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

    pair<int,int> findLookheadPoint(int x, int y){
        int id = findNearestPoint(x, y);
        if (id > curPoint)
            curPoint = id;
        int cur = two2one(x, y);
        while(curPoint + 1 < path.size() && dis(cur, path[curPoint]) < l){
            ++ curPoint;
        }
        return one2two(path[curPoint]);
    }

    void drawpath(int w, int h, Display * display){
        for (auto i : path){
            auto [x, y] = one2two(i);
            auto [nX, nY] = pic2real(y, x, w, h);
            display->drawPixel(nX, nY);
        }
    }
};

int main()
{
    Path maze("../Move/example.txt");
    maze.setLookhead(lookDis);


    Supervisor *su = new Supervisor();
    int timeStep = (int)su->getBasicTimeStep();
    Display *display = su->getDisplay("display");
    Node * robot = su->getFromDef("Robot");

    display->setOpacity(1);

    int width = display->getWidth();
    int height = display->getHeight();

    display->setColor(BLUE);
    maze.drawpath(width, height, display);
    display->setColor(RED);

    ImageRef* to_store = NULL;
    while (su->step(timeStep) != -1)
    {

        // 获取小车位置
        const double X = robot->getPosition()[0];
        const double Y = robot->getPosition()[1];
        const double Z = robot->getPosition()[2];

        double real = 2;

        // 模拟行为

        auto [x, y] = real2pic(Y, X);

        auto [ry, rx] = pic2real(x, y, width, height);

        auto [nx, ny] = maze.findLookheadPoint(x, y);

        auto [nY, nX] = pic2real(nx, ny, width, height);
        
        if (to_store != NULL) // 恢复图片，去除原来的圆圈
            display->imagePaste(to_store, 0, 0);
        // 绘制目标点
        display->drawPixel(nX, nY);

        if (to_store != NULL)
            display->imageDelete(to_store);

        // 保存当前图片
        to_store = display->imageCopy(0, 0, width, height);

        display->setColor(GREEN);
        // 绘制圆圈
        display->drawOval(rx, ry, real, real);
        display->setColor(RED);

    }

    delete su;
    return 0;
}
