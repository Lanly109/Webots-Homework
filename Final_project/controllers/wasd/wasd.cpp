#include <webots/Display.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <vector>

using namespace std;
using namespace webots;

const double pi = acos(-1);
const double eps = 1e-6;
const int up = 20;
const int update_up = 16;
// 原点在左上角，x y意义同坐标系下的意义
/* world 1 */
// const double WORLD_WIDTH = 5;
// const double WORLD_HEIGHT = 5;
// const int obs_r = 1;
// const int start_x = 90, start_y = 5, end_x = 5, end_y = 90;
/* --------------------------------------- */
/* world 2 */
// const double WORLD_WIDTH = 4.5;
//const double WORLD_HEIGHT = 6;
// const int obs_r = 1;
// const int start_x = 90, start_y = 5, end_x = 5, end_y = 90;
/* --------------------------------------- */
/* world 3 */
const double WORLD_WIDTH = 10;
const double WORLD_HEIGHT = 10;
const int obs_r = 1;
const int start_x = 90, start_y = 5, end_x = 5, end_y = 90;
/* --------------------------------------- */

#define RED 0xBB2222
#define GREEN 0x22BB11
#define BLUE 0x2222BB
#define BLACK 0x000000

#define NEW 0
#define OPEN 1
#define CLOSED 2

int occupy[1000][1000] = {};
int stop[1000][1000] = {};

class Coordinates
{
public:
    double x, y;
    Coordinates(double xx, double yy) : x(xx), y(yy) {}
    Coordinates() : x(0), y(0) {}
    bool operator==(const Coordinates &c) { return x == c.x && y == c.y; }
    bool operator!=(const Coordinates &c) { return !(this->operator==(c)); }
    bool operator==(const Coordinates &c) const
    {
        return (x == c.x && y == c.y);
    }
};

/**
 * @brief PID控制器，用于PID控制算法
 * 
 */
class PID
{
public:
    Coordinates SetPoint; //  设定目标 Desired Value
    Coordinates SumError; //	误差累计

    double Proportion; //  比例常数 Proportional Const
    double Integral;   //  积分常数 Integral Const
    double Derivative; //  微分常数 Derivative Const

    Coordinates LastError; //  Error[-1]
    Coordinates PrevError; //  Error[-2]

    PID()
    {
        Proportion = Integral = Derivative = SetPoint.x = SetPoint.y =
            SumError.x = SumError.y = LastError.x = LastError.y = PrevError.x =
                PrevError.y = 0;
    }

    /**
     * @brief 设置期望点
     * 
     * @param setpoint 期望点
     */
    void PID_SetPoint(Coordinates *setpoint)
    {
        SetPoint.x = setpoint->x;
        SetPoint.y = setpoint->y;
    }
    void init()
    {
        SetPoint.x = SetPoint.y = SumError.x = SumError.y = LastError.x =
            LastError.y = PrevError.x = PrevError.y = 0;

        Proportion = 0.97;
        Integral = 0.05;
        Derivative = 50;
    }

    /**
     * @brief x方向PID算法
     * 
     * @param CurPoint 当前位置
     * @return double PID反馈量
     */
    double PID_x_PosLocCalc(Coordinates *CurPoint)
    {
        double iError_x, dError_x;

        iError_x = SetPoint.x - CurPoint->x; // 偏差
        SumError.x += iError_x;              // //累计偏差，用于积分
        if (SumError.x > 10.0)               //积分限幅10
            SumError.x = 10.0;
        else if (SumError.x < -10.0)
            SumError.x = -10.0;
        dError_x = iError_x - LastError.x; // 当前微分
        LastError.x = iError_x;

        return (double)(Proportion * iError_x   // 比例项
                        + Integral * SumError.x // 积分项
                        + Derivative * dError_x);
    }

    /**
     * @brief y方向PID算法
     * 
     * @param CurPoint 当前位置
     * @return double PID反馈量
     */
    double PID_y_PosLocCalc(Coordinates *CurPoint)
    {
        double iError_y, dError_y;

        iError_y = SetPoint.y - CurPoint->y; // 偏差
        SumError.y += iError_y;              // //累计偏差，用于积分
        if (SumError.y > 10.0)               //积分限幅10
            SumError.y = 10.0;
        else if (SumError.y < -10.0)
            SumError.y = -10.0;
        dError_y = iError_y - LastError.y; // 当前微分
        LastError.y = iError_y;

        return (double)(Proportion * iError_y   // 比例项
                        + Integral * SumError.y // 积分项
                        + Derivative * dError_y);
    }
};

vector<Coordinates *> path; // #TODO：存放规划后路径

// 返回一个浮点数的符号
int sign(double x)
{
    if (x > eps)
        return 1;
    if (x < -eps)
        return -1;
    return 0;
}

// 激光雷达感知过程
class process
{
    int **cnt; // 用于统计

    int n, m; // 栅栏地图大小

    double st; //常数 -pi / 2;

    double real_n, real_m; // 真实地图大小

    double unit; //单元数

    double max_range; //雷达最大范围

public:
    // 真实地图坐标转化为栅栏地图坐标
    pair<int, int> real_to_pic(double x, double y)
    {
        auto xx = (x + real_m / 2) / real_m;
        xx = max(min(xx, 0.99), 0.0);
        auto yy = -(y - real_n / 2) / real_n;
        yy = max(min(yy, 0.99), 0.0);
        int nxx = m * xx;
        if (nxx == m)
            -- nxx;
        int nyy = n * yy;
        if (nyy == n)
            -- nyy;
        return make_pair(nxx, nyy);
    }

    // 初始化内存
    process(int resolve,
            double max_range = 1,
            int n = 512,
            int m = 512,
            double real_n = 10,
            double real_m = 10)
        : max_range(max_range), n(n), m(m), real_n(real_n), real_m(real_m)
    {
        st = -pi / 2;
        unit = 2 * pi / resolve;
        cnt = new int *[n];
        for (int i = 0; i < n; ++i)
        {
            cnt[i] = new int[m];
            memset(cnt[i], 0, sizeof(int) * m);
        }
    }

    // 释放内存
    ~process()
    {
        for (int i = 0; i < m; ++i)
        {
            delete[] cnt[i];
        }
        delete[] cnt;
    }

    // 增加计数, 计数大于一定阈值才判定为障碍
    void add(int total, const float *dis, double x, double y)
    {
        for (int i = 0; i < total; ++i)
        {
            if (sign(dis[i] - max_range) == 0)
                continue;
            auto [ny, nx] = real_to_pic(x + dis[i] * cos(st - i * unit),
                                        y + dis[i] * sin(st - i * unit));
            if (nx >= 0 && nx < n && ny >= 0 && ny < m)
                cnt[nx][ny]++;
        }
    }

    // 用红色绘制障碍
    void draw(Display *display)
    {
        display->setColor(RED);
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < m; ++j)
            {
                if (occupy[i][j])
                    display->drawPixel(i, j);
            }
        }
    }

    // 通过计数更新障碍
    void update(Display *display)
    {
        display->setColor(RED);
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < m; ++j)
            {
                if (cnt[i][j] >= up)
                {
                    occupy[i][j] = 1;
                    display->drawPixel(j, i);
                }
            }
        }

        for (int i = 0; i < n; ++i)
            memset(cnt[i], 0, sizeof(int) * m);
    }

    // 用蓝色绘制曾经到达过的点
    void draw_pos(Display *display, const double *pos)
    {
        display->setColor(BLUE);
        auto [y, x] = real_to_pic(pos[0], pos[1]);
        display->drawPixel(y, x);
    }
};

class Dstar
{
public:
    int start_x, start_y, end_x, end_y, cur_x, cur_y;// 开始位置, 结束位置, 当前位置

    int m, n;// 地图大小

    int stay, flag, turn;// 用来解决卡墙问题的变量

    int **state, **nxt_x, **nxt_y;// 节点当前状态

    double **hscore, **kscore;// h值和k值

    vector<pair<int, int>> openlist;// OPEN list

    // 八个方向的邻居
    int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1},
        dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

    vector<pair<int, int>> path;// 记录路径

    // 内存初始化
    Dstar(int start_x, int start_y, int end_x, int end_y, int m, int n)
        : start_x(start_x),
          start_y(start_y),
          end_x(end_x),
          end_y(end_y),
          cur_x(start_x),
          cur_y(start_y),
          m(m),
          n(n)
    {
        path.clear();
        state = new int *[n];
        nxt_x = new int *[n];
        nxt_y = new int *[n];
        hscore = new double *[n];
        kscore = new double *[n];
        for (int i = 0; i < n; ++i)
        {
            state[i] = new int[m];
            nxt_x[i] = new int[m];
            nxt_y[i] = new int[m];
            hscore[i] = new double[m];
            kscore[i] = new double[m];
            // 设置为New
            memset(state[i], 0, sizeof(int) * m);
            memset(nxt_x[i], 0, sizeof(int) * m);
            memset(nxt_y[i], 0, sizeof(int) * m);
        }
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < m; j++)
            {
                hscore[i][j] = calDis(j, i, end_y, end_x);
                kscore[i][j] = hscore[i][j];
            }
        }
        nxt_x[end_y][end_x] = end_x;
        nxt_y[end_y][end_x] = end_y;
        openlist.push_back(make_pair(end_y, end_x));
        stay = 0;
        flag = 0;
        turn = 0;
    }

    // 回收内存
    ~Dstar()
    {
        for (int i = 0; i < m; ++i)
        {
            delete[] hscore[i];
            delete[] kscore[i];
        }
        delete[] hscore;
        delete[] kscore;
    }

    // 得到OPEN list中最小k值
    double get_mink()
    {
        if (!openlist.size())
            return -1;
        double res = 1e6;
        for (int i = 0; i < (int)openlist.size(); i++)
        {
            res = min(res, kscore[openlist[i].first][openlist[i].second]);
        }
        return res;
    }

    // 得到OPEN list中最小k值对应的节点
    pair<int, int> get_min_state()
    {
        if (!openlist.size())
            return make_pair(-1, -1);
        double mink = get_mink();
        pair<int, int> res = make_pair(-1, -1);
        for (int i = 0; i < (int)openlist.size(); i++)
        {
            if (mink == kscore[openlist[i].first][openlist[i].second])
            {
                res = openlist[i];
                openlist.erase(openlist.begin() + i);
                return res;
            }
        }
        return res;
    }

    // 向 OPEN list 插入更新的节点
    void insert(int x, int y, double h)
    {
        double k;
        if (state[y][x] == NEW)
        {
            k = h;
        }
        if (state[y][x] == OPEN)
        {
            k = min(kscore[y][x], h);
        }
        if (state[y][x] == CLOSED)
        {
            k = min(hscore[y][x], h);
        }
        kscore[y][x] = k;
        hscore[y][x] = h;
        state[y][x] = OPEN;
        openlist.push_back(make_pair(y, x));
    }

    // 节点处理函数
    int process_state()
    {
        // 取所有节点中 k 值最小的
        pair<int, int> tmp = get_min_state();
        int y = tmp.first, x = tmp.second;
        // OPEN list 为空，结束
        if (y == -1 && x == -1)
            return -1;

        //获得该点k值，并将该点弹出Openlist队列
        double h = hscore[y][x], k = kscore[y][x];
        state[y][x] = CLOSED;

        // 该点 X 处于 RAISE 状态
        if (k < h)
        {
            //遍历邻域节点 Y
            for (int i = 0; i < 8; i++)
            {
                if (x + dx[i] < 0 || x + dx[i] >= m)
                    continue;
                if (y + dy[i] < 0 || y + dy[i] >= n)
                    continue;
                int extend_x = x + dx[i], extend_y = y + dy[i];
                double cost = calDis(x, y, extend_x, extend_y);

                // 遇到障碍重新设置其 h 值和 k 值
                if (stop[extend_y][extend_x] || obstacle(extend_x, extend_y))
                {
                    hscore[extend_y][extend_x] = kscore[extend_y][extend_x] =
                        1e6;
                    stop[extend_y][extend_x] = 1;
                }

                //Y 不处于RAISE 状态，且用其更新x后，X 的 h 值更小, 那么就更新 X 的父节点及其 h 值
                if (hscore[extend_y][extend_x] <= k &&
                    hscore[y][x] > hscore[extend_y][extend_x] + cost)
                {
                    nxt_x[y][x] = extend_x, nxt_y[y][x] = extend_y;
                    hscore[y][x] = hscore[extend_y][extend_x] + cost;
                }
            }
        }

        // 该点 X 处于 LOWER 状态
        if (k == h)
        {
            // 遍历邻域节点Y
            for (int i = 0; i < 8; i++)
            {
                if (x + dx[i] < 0 || x + dx[i] >= m)
                    continue;
                if (y + dy[i] < 0 || y + dy[i] >= n)
                    continue;
                int extend_x = x + dx[i], extend_y = y + dy[i];
                int back_flag = (nxt_x[extend_y][extend_x] == x &&
                                 nxt_y[extend_y][extend_x] == y);
                double cost = calDis(x, y, extend_x, extend_y);

                // 分以下三种情况
                // Y是首次添加到OPEN list并处理
                // Y是X的子节点，并且其h值不需要改变
                // Y不是X的子节点，并且其h值可以变得更小
                if (state[extend_y][extend_x] == NEW ||
                    (back_flag &&
                     hscore[extend_y][extend_x] != hscore[y][x] + cost) ||
                    (!back_flag &&
                     hscore[extend_y][extend_x] > hscore[y][x] + cost))
                {
                    // 将X作为Y的父节点，修改其h值，并将Y点添加到Openlist中
                    nxt_x[extend_y][extend_x] = x,
                    nxt_y[extend_y][extend_x] = y;
                    insert(extend_x, extend_y, hscore[y][x] + cost);
                }
            }
        }
        else
        {
            // 遍历邻域节点Y
            for (int i = 0; i < 8; i++)
            {
                if (x + dx[i] < 0 || x + dx[i] >= m)
                    continue;
                if (y + dy[i] < 0 || y + dy[i] >= n)
                    continue;
                int extend_x = x + dx[i], extend_y = y + dy[i];
                int back_flag = (nxt_x[extend_y][extend_x] == x &&
                                 nxt_y[extend_y][extend_x] == y);
                double cost = calDis(x, y, extend_x, extend_y);

                // 分以下两种情况
                // Y是首次添加到OPEN list并处理
                // Y是X的子节点，并且其h值不需要改变
                if (state[extend_y][extend_x] == NEW ||
                    (back_flag &&
                     hscore[extend_y][extend_x] != hscore[y][x] + cost))
                {
                    // 将X作为Y的父节点，修改其h值，并将Y点添加到Openlist中
                    nxt_x[extend_y][extend_x] = x,
                    nxt_y[extend_y][extend_x] = y;
                    insert(extend_x, extend_y, hscore[y][x] + cost);
                }
                else
                {
                    //Y不是X的子节点，并且其h值可以变得更小
                    if (!back_flag &&
                        hscore[extend_y][extend_x] > hscore[y][x] + cost)
                    {
                        //修改X的h值，并将其点添加到Openlist中
                        insert(x, y, hscore[y][x]);
                    }
                    //Y不是X的子节点，并且其h值可以变得更小, 并且Y不在Openlist中，且h值处于上升状态
                    else if (!back_flag &&
                             hscore[y][x] >
                                 hscore[extend_y][extend_x] + cost &&
                             state[extend_y][extend_x] == CLOSED &&
                             hscore[extend_y][extend_x] > k)
                    {
                        //修改Y的h值，并将其点添加到Openlist中
                        insert(extend_x, extend_y, hscore[extend_y][extend_x]);
                    }
                }
            }
        }
        //返回该点k值
        return get_mink();
    }

    // 绘制路径
    void draw(Display *display)
    {
        display->setColor(GREEN);
        for (int i = 0; i < (int)path.size(); i++)
        {
            if (occupy[path[i].second][path[i].first])
                continue;
            display->drawPixel(path[i].first, path[i].second);
        }
    }

    // 清除上次绘制的路径, 实现路径的动态变化
    void drawclear(Display *display)
    {
        display->setColor(BLACK);
        for (int i = 0; i < (int)path.size(); i++)
        {
            if (occupy[path[i].second][path[i].first])
                continue;
            display->drawPixel(path[i].first, path[i].second);
        }
    }

    // 生成路径
    void generate_path(Display *display)
    {
        // 遍历得到路径
        path.clear();
        int x = cur_x, y = cur_y;
        int cnt = 0;
        while (1)
        {
            if (cnt >= 10000)
                break;
            cnt += 1;
            path.push_back(make_pair(x, y));
            if (x == end_x && y == end_y)
                break;
            if (x == -1 || y == -1)
                break;
            int tmp_x = nxt_x[y][x], tmp_y = nxt_y[y][x];
            x = tmp_x, y = tmp_y;
            if (x == end_x && y == end_y)
                break;
        }
        // 遇到卡墙情况, 进行时间计数
        if (obstacle(cur_x, cur_y))
        {
            stay++;
        }
        // 超过一定时间进行调整位置
        if (stay >= 25)
        {
            flag = 15;
            stay = 0;
            turn = !turn;
        }
        // 按照路径反方向调整位置
        if (flag)
        {
            flag--;
            stay = 0;
            vector<pair<int, int>> tmp_path;
            for (int i = 0; i < 10; i++)
            {
                if (turn)
                {
                    tmp_path.push_back(make_pair(2 * cur_x - path[i * 2].first, cur_y));
                }
                else
                {
                    tmp_path.push_back(make_pair(cur_x, 2 * cur_y - path[i * 2].second));
                }
            }
            path = tmp_path;
        }
        // 绘制路径
        draw(display);
    }

    // 初始化路径
    void work(Display *display)
    {
        drawclear(display);
        kscore[end_y][end_x] = 0;
        state[cur_y][cur_x] = NEW;
        while (1)
        {
            if (state[cur_y][cur_x] == CLOSED)
            {
                break;
            }
            process_state();
        }
        generate_path(display);
    }

    // 获取路径
    vector<pair<int, int>> get_path() { return path; }

    // 遇到障碍物时进行路径更新
    void update(int cur_x, int cur_y, Display *display)
    {
        drawclear(display);
        this->cur_x = cur_x, this->cur_y = cur_y;
        kscore[end_y][end_x] = 0;
        int x = cur_x, y = cur_y;
        int cnt = 0;
        while (1)
        {
            if (cnt >= 1000)
                break;
            cnt += 1;
            if (x == end_x && y == end_y)
                break;
            if (x == -1 || y == -1)
                break;
            // 遇到障碍物重新计算cost
            if (stop[nxt_y[y][x]][nxt_x[y][x]] ||
                obstacle(nxt_x[y][x], nxt_y[y][x]))
            {
                stop[nxt_y[y][x]][nxt_x[y][x]] = 1;
                modify(x, y);
                continue;
            }
            int tmp_x = nxt_x[y][x], tmp_y = nxt_y[y][x];
            x = tmp_x, y = tmp_y;
            if (x == end_x && y == end_y)
                break;
        }
        generate_path(display);
    }

    // 重新计算节点cost
    void modify_cost(int x, int y)
    {
        if (state[y][x] == CLOSED)
        {
            double cost = calDis(x, y, nxt_x[y][x], nxt_y[y][x]);
            double tmp = hscore[y][x] + cost;
            insert(x, y, tmp);
        }
    }

    // 重新计算节点cost, 并递归更新受影响的其他节点
    void modify(int x, int y)
    {
        modify_cost(x, y);
        int cnt = 0;
        while (1)
        {
            if (cnt >= 10)
                break;
            cnt += 1;
            double mink = process_state();
            if (mink >= hscore[y][x])
                break;
        }
    }

    // 判断一个点是否在障碍上
    int obstacle(int x, int y)
    {
        if (occupy[y][x])
            return 1;
        if (abs(end_x - x) <= 2 && abs(end_y - y) <= 2)
            return 0;
        for (int i = -obs_r; i <= obs_r; i++)
        {
            for (int j = -obs_r; j <= obs_r; j++)
            {
                if (x + i < 0 || x + i >= m)
                    continue;
                if (y + j < 0 || y + j >= n)
                    continue;
                if (occupy[y + j][x + i])
                    return 1;
            }
        }
        return 0;
    }

    // 计算两点之间的距离
    double calDis(int x1, int y1, int x2, int y2)
    {
        // 遇到了障碍
        if (stop[y1][x1] || stop[y2][x2] || obstacle(x1, y1) ||
            obstacle(x2, y2))
            return 1e6;
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }
};

/**
 * @brief 将目标路径存储到Path中，传给运动控制部分
 * 
 * @param dstar 规划控制器
 * @param path 路径存储结构
 */
void loadPath(Dstar *dstar, vector<Coordinates *> *path)
{
    (*path).clear();
    for (int i = 0; i < dstar->path.size(); ++i)
    {
        Coordinates *ls =
            new Coordinates(dstar->path[i].first, dstar->path[i].second);
        (*path).push_back(ls);
    }
}

char action_x(double pid, int cur) {
    double d = pid - cur;
    if (abs(d) <= 1)  return 'N';
    char ret;
    ret = d > 0 ? 'S' : 'W';
    return ret;
}

char action_y(double pid, int cur) {
    double d = pid - cur;
    if (abs(d) <= 1)  return 'N';
    char ret;
    ret = d > 0 ? 'A' : 'D';
    return ret;
}

int main()
{
    Robot *robot = new Robot();

    int timeStep = (int)robot->getBasicTimeStep();
    cout << timeStep << endl;

    Keyboard keyboard;
    keyboard.enable(1);

    Lidar *lidar = robot->getLidar("lidar");
    lidar->enable(1);
    lidar->enablePointCloud();
    const int lidar_point_count = lidar->getHorizontalResolution();

    GPS *gps = robot->getGPS("gps");
    gps->enable(1);

    InertialUnit *inertial = robot->getInertialUnit("yaw");
    inertial->enable(1);

    Display *display = robot->getDisplay("display");

    display->setOpacity(1);

    int width = display->getWidth();
    int height = display->getHeight();

    process slam(lidar_point_count, lidar->getMaxRange(), height, width,
                 WORLD_HEIGHT, WORLD_WIDTH);

    // Dstar 算法
    Dstar dstar(start_x, start_y, end_x, end_y, height, width);
    // 生成初始地图
    dstar.work(display);

    loadPath(&dstar, &path);

    Motor *motors[4];

    char wheelsNames[4][8] = {"motor1", "motor2", "motor3", "motor4"};
    double speed1[4];
    double speed2[4];
    double speed3[4];

    for (int i = 0; i < 4; i++)
    { // 初始化电机和距离传感器
        motors[i] = robot->getMotor(wheelsNames[i]);
        motors[i]->setPosition(std::numeric_limits<double>::infinity());
        motors[i]->setVelocity(0.0);
        speed1[i] = 0;
        speed2[i] = 0;
        speed3[i] = 0;
    }

    const double velocity = 50;
    const double velocity2 = 10;

    double speed_forward[4] = {velocity, velocity, velocity, velocity};
    double speed_backward[4] = {-velocity, -velocity, -velocity, -velocity};
    double speed_leftward[4] = {velocity, -velocity, velocity, -velocity};
    double speed_rightward[4] = {-velocity, velocity, -velocity, velocity};

    double speed_leftCircle[4] = {velocity2, -velocity2, -velocity2, velocity2};
    double speed_rightCircle[4] = {-velocity2, velocity2, velocity2,
                                   -velocity2};

    const double front = 1.57;

    int count = 0;

    int idx = 0;

    PID pos_pid; // PID manager
    pos_pid.init(); // 对PID控制器进行初始化

    double xx = 1.95;
    double yy = 1.92;


    for (int i = 0; i < path.size(); i++)
    {
        cout << path[i]->x << " " << path[i]->y << endl;
    }

    while (robot->step(timeStep) != -1)
    {
        const double *yaw = inertial->getRollPitchYaw();

        if (fabs(yaw[0] - front) >= 0.05)
        {
            int dir = (yaw[0] < front && yaw[0] > -front ? -1 : 1);
            for (int i = 0; i < 4; ++i)
            {
                speed3[i] = speed_rightCircle[i] * dir;
                motors[i]->setVelocity(speed3[i]);
            }
            continue;
        }
        else
        {
            for (int i = 0; i < 4; ++i)
                speed3[i] = 0;
        }
        count++;
        const double *pos = gps->getValues();
        const float *dis = lidar->getRangeImage();

        // 计算障碍的计数
        slam.add(lidar_point_count, dis, pos[0], pos[1]);

        // for (int i = 0; i < lidar_point_count; ++i)
        // {
        //     printf("%.4f ", dis[i]);
        // }
        // puts("");

        // 进行地图的更新和路径的重新规划
        if (count == update_up)
        {
            count = 0;
            slam.update(display);// 更新地图
            pair<int, int> tmp = slam.real_to_pic(pos[0], pos[1]);// 当前位置
            dstar.update(tmp.first, tmp.second, display);// 重新规划路径

            // 转储Path
            loadPath(&dstar, &path);

            idx = 0; // 下标重置
            pos_pid.init(); // 初始化PID控制器
        }

        slam.draw_pos(display, pos);
        printf("position & yaw: %.2lf %.2lf %.2lf %.2lf\n", pos[0], pos[1],
               pos[2], yaw[0]);

        // 设置PID目标值
        pos_pid.PID_SetPoint(path[idx]);

        double cx = pos[0];
        double cy = pos[1];

        // 记录当前位置
        auto [curx, cury] = slam.real_to_pic(pos[0], pos[1]);

        Coordinates cur(curx, cury);

        // 根据当前位置计算PID反馈量，并加到当前位置中
        double pid_x = pos_pid.PID_x_PosLocCalc(&cur) + cur.x;
        double pid_y = pos_pid.PID_y_PosLocCalc(&cur) + cur.y;

        int keyValue1;
        int keyValue2;

        // 根据PID反馈量进行决策
        cout << "cur: " << curx << " " << cury << endl;
        keyValue1 = action_x(pid_x, curx);
        keyValue2 = action_y(pid_y, cury);

        cout << "action: " << char(keyValue1) << ", " << char(keyValue2)
             << endl;

        cout << "gps_car Value X: " << curx << " Y: " << cury
             << " Z: " << pos[2] << endl;

        cout << "pid Value X: " << pid_x << " Y: " << pid_y << endl;

        cout << "target Value X: " << path[idx]->x << " Y: " << path[idx]->y
             << endl;

        for (int i = 0; i < 4; i++)
        {
            speed1[i] = speed2[i] = 0;
        }

        if (keyValue1 == 'W')
        {
            for (int i = 0; i < 4; i++)
            {
                speed1[i] = speed_forward[i];
            }
        }
        else if (keyValue1 == 'S')
        {
            for (int i = 0; i < 4; i++)
            {
                speed1[i] = speed_backward[i];
            }
        }
        else if (keyValue1 == 'A')
        {
            for (int i = 0; i < 4; i++)
            {
                speed1[i] = speed_leftward[i];
            }
        }
        else if (keyValue1 == 'D')
        {
            for (int i = 0; i < 4; i++)
            {
                speed1[i] = speed_rightward[i];
            }
        }
        else if (keyValue1 == 'Q')
        {
            for (int i = 0; i < 4; i++)
            {
                speed1[i] = speed_leftCircle[i];
            }
        }
        else if (keyValue1 == 'E')
        {
            for (int i = 0; i < 4; i++)
            {
                speed1[i] = speed_rightCircle[i];
            }
        }
        else
        {
            for (int i = 0; i < 4; i++)
            {
                speed1[i] = 0;
            }
        }
        if (keyValue2 == 'W')
        {
            for (int i = 0; i < 4; i++)
            {
                speed2[i] = speed_forward[i];
            }
        }
        else if (keyValue2 == 'S')
        {
            for (int i = 0; i < 4; i++)
            {
                speed2[i] = speed_backward[i];
            }
        }
        else if (keyValue2 == 'A')
        {
            for (int i = 0; i < 4; i++)
            {
                speed2[i] = speed_leftward[i];
            }
        }
        else if (keyValue2 == 'D')
        {
            for (int i = 0; i < 4; i++)
            {
                speed2[i] = speed_rightward[i];
            }
        }
        else
        {
            for (int i = 0; i < 4; i++)
            {
                speed2[i] = 0;
            }
        }
        cout << "speed : ";
        for (int i = 0; i < 4; i++)
        {
            motors[i]->setVelocity(speed1[i] + speed2[i] + speed3[i]);
            cout << speed1[i] + speed2[i] + speed3[i] << ", ";
        }
        cout << endl;
        if (abs(end_y - cury) <= 1 && abs(end_x - curx) <= 1)
        {
            cout << endl
                 << "============ Arrive Goal ! ============" << endl;
            for (int i = 0; i < 4; i++)
            {
                motors[i]->setVelocity(0);
            }
            break;
        }
        if ((abs(pid_y - cury) <= 3 && abs(pid_x - curx) <= 3) &&
            idx < path.size())
        {
            idx++;
        }
    }
    delete robot;
    return 0;
}
