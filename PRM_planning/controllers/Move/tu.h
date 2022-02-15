#include <opencv2/opencv.hpp>
#include <queue>
#include <set>
#include <string>

using namespace std;

const int ST = 76;      // 起始点颜色
const int ED = 36;      // 终点颜色
const int REC = 15;     // 撒点远离障碍物的切比雪夫距离
const double neiDis = 35; // 邻居判断阈值
const int disX = 20;    // 原图分块大小
const int disY = 20;    // 原图分块大小

class tuP{
    int cols, rows;
    int sx, sy;
    int ex, ey;
    int l;
    set<int> obs;   // 记录障碍物的一维坐标
    vector<pair<int,int>> point;    // 记录点坐标
    vector<vector<int>> edge;       // 记录连边
    cv::Mat maze;

    public:
    tuP(){
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

    tuP(string s){
        sx = ex = -1; 
        maze = cv::imread(s.c_str(), cv::IMREAD_COLOR);
        cols = maze.cols;
        rows = maze.rows;
        for(int i = 0; i < rows; ++ i)
            for(int j = 0; j < cols; ++ j){ // 遍历图
                int b = maze.at<cv::Vec3b>(i, j)[0];
                int g = maze.at<cv::Vec3b>(i, j)[0];
                int r = maze.at<cv::Vec3b>(i, j)[0];
                if (b == ST && sx == -1){ // 起点
                    sx = i;
                    sy = j;
                }
                if (b == ED){ // 终点
                    ex = i;
                    ey = j;
                }
                if (b != ST && b != ED && b != 255) // 障碍物
                    obs.insert(two2one(i, j));
            }
        if (sx == -1){
            printf("ERROR: CANNOT FIND Start Point!\n");
            exit(0);
        }else{
            point.push_back({sx, sy});
            printf("INFO: Find Start Point: %d %d\n", sx, sy);
        }
        if (ex == -1){
            printf("ERROR: CANNOT FIND Target Point!\n");
            exit(0);
        }else{
            point.push_back({ex, ey});
            printf("INFO: Find Target Point: %d %d\n", ex, ey);
        }
    }

    inline bool valid(int x, int y){ // 判断当前点是否合法，即是否越界或者是否在障碍物边缘
        if (x < 0 && x >= rows)
            return false;
        if (y < 0 && y >= cols)
            return false;
        for(int i = -REC; i <= REC; ++ i)
            for(int j = -REC; j <= REC; ++ j)
                if (obs.find(two2one(x + i, y + j)) != obs.end())
                    return false;
        return true;
    }

    bool checkLine(int x1, int y1, int x2, int y2){ // 判断一条直线上是否有障碍物
        double k = 0.01;
        for(; k <= 1; k += 0.01)
            if (!valid(x1 + (x2 - x1) * k, y1 + (y2 - y1) * k))
                return false;
        return true;
    }

    void randGenPoint(){ // 随机生成点
        for(int i = 0; i < rows; i += disX)
            for(int j = 0; j < cols; j += disY){
                int attempCnt = 25;
                while(attempCnt--){
                    int x = rand() % disX;
                    int y = rand() % disY;
                    if (valid(x + i, j + y)){
                        point.push_back({x + i, y + j});
                        break;
                    }
                }
            }
    }

    void connectNei(){ // 邻居连边
        edge.resize(rows * cols);
        for(auto [x, y] : point){
            for(auto [nx, ny] : point){
                if (nx == x && ny == y)
                    continue;
                if (dis(x, y, nx, ny) < neiDis && checkLine(x, y, nx, ny)){
                    edge[two2one(x, y)].push_back(two2one(nx, ny));
                    edge[two2one(nx, ny)].push_back(two2one(x, y));
                }
            }
        }
    }

    vector<int> path; // 记录最优路径的点的一维坐标

    void findPath(){ // 运用dijkstra寻找一条最短路
        randGenPoint();
        connectNei();

        vector<int> pre(cols * rows); // 记录点的前驱
        vector<bool> vis(cols * rows, false);
        double oo = 1e9 + 7;
        vector<double> distance(cols * rows, oo);
        priority_queue<pair<double, int>> team;
        team.push({0, two2one(sx, sy)});
        distance[two2one(sx, sy)] = 0;
        while(!team.empty()){ // dijkstra主要部分
            auto [_, u] = team.top();
            team.pop();
            if (vis[u])
                continue;
            vis[u] = true;
            for(auto v : edge[u]){
                double w = dis(u, v);
                if (distance[v] > distance[u] + w){
                    distance[v] = distance[u] + w;
                    pre[v] = u;
                    team.push({-distance[v], v});
                }
            }
        }
        if (distance[two2one(ex, ey)] == oo){
            printf("WARNING: CANNOT Find a path from start point to target point!\n");
            exit(0);
        }
        for(int u = two2one(ex, ey); u != two2one(sx, sy); u = pre[u])
            path.push_back(u);
        path.push_back(two2one(sx, sy));
        reverse(path.begin(), path.end()); // 反转，使第一个点为起点
    }

    // 以下函数未用到，因为一开始代码未分开，想着找完最短路就可以直接开始寻路，但后来发现无法通过编译（链接出错），因此复制了以下代码到Move.cpp中

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

    void clear(){
        curPoint = -1;
    }

    // 绘制采样点

    void drawPoint(){
        for(auto [y, x] : point){
            cv::rectangle(maze, cv::Point_<int>(x, y), cv::Point_<int>(x + 1, y + 1), cv::Scalar_<double>(255, 0, 0), -1);
        }
    }

    // 绘制连边

    void drawNei(){
        for(int u = 0 ; u < edge.size(); ++ u)
            for(auto v : edge[u]){
                auto [y, x] = one2two(u);
                auto [ny, nx] = one2two(v);
                cv::line(maze, cv::Point_<int>(x, y), cv::Point_<int>(nx, ny), cv::Scalar_<double>(0,0,255), 1);
        }
    }

    // 绘制最短路

    void drawPath(){
        if (path.empty())
            findPath();
        for(int i = 0; i < path.size() - 1; ++ i){
            auto [y, x] = one2two(path[i]);
            auto [ny, nx] = one2two(path[i + 1]);
            cv::line(maze, cv::Point_<int>(x, y), cv::Point_<int>(nx, ny), cv::Scalar_<double>(255,255,0), 3);
        }
    }

    // 保存图片

    void savePic(string s){
        cv::imwrite(s.c_str(), maze);
    }

    // 保存路径

    void savePath(string s){
        FILE* fp = fopen(s.c_str(), "w");
        fprintf(fp, "%d %d %lu\n", rows, cols, path.size());
        for(auto i : path)
            fprintf(fp, "%d\n", i);
    }


};
