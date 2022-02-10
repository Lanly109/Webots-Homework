#include <opencv2/opencv.hpp>
#include <queue>
#include <set>
#include <string>
#include <vector>

using namespace std;

const int ST = 76;      // 起始点颜色
const int ED = 36;      // 终点颜色
const int REC = 15;     // 撒点远离障碍物的切比雪夫距离
const int stepDistance = 5; // 步长距离
const int rewriteDistance = 30; // 重布线的点的距离
const int fatherDistance = 15;  // 父亲选择的距离
const int targetDistance = 10;  // 距离终点的距离

class tuP{
    int cols, rows;
    int sx, sy;
    int ex, ey;
    int l;
    set<int> obs;   // 记录障碍物的一维坐标
    vector<set<int>> edge;          // 父亲指向儿子的边
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
                if (b != ST && b != ED && b != 255){ // 障碍物
                    for(int x = -REC; x <= REC; ++ x)
                        for(int y = -REC; y <= REC; ++ y)
                            obs.insert(two2one(i + x, j + y));
                }
            }
        if (sx == -1){
            printf("ERROR: CANNOT FIND Start Point!\n");
            exit(0);
        }else{
            printf("INFO: Find Start Point: %d %d\n", sx, sy);
        }
        if (ex == -1){
            printf("ERROR: CANNOT FIND Target Point!\n");
            exit(0);
        }else{
            printf("INFO: Find Target Point: %d %d\n", ex, ey);
        }
    }

    inline bool valid(int x, int y){ // 判断当前点是否合法，即是否越界或者是否在障碍物边缘
        if (x < 0 || x >= rows)
            return false;
        if (y < 0 || y >= cols)
            return false;
        return obs.find(two2one(x, y)) == obs.end();
        // 大大的优化，速度提升了REC*REC倍
        //
        // for(int i = -REC; i <= REC; ++ i)
        //     for(int j = -REC; j <= REC; ++ j)
        //         if (obs.find(two2one(x + i, y + j)) != obs.end())
        //             return false;
        // return true;
    }

    bool checkLine(int x1, int y1, int x2, int y2){ // 判断一条直线上是否有障碍物
        double k = 0.01;
        for(; k <= 1; k += 0.01)
            if (!valid(x1 + (x2 - x1) * k, y1 + (y2 - y1) * k))
                return false;
        return true;
    }

    bool checkLine(int p1, int p2){ // 判断一条直线上是否有障碍物
        auto [x1, y1] = one2two(p1);
        auto [x2, y2] = one2two(p2);
        return checkLine(x1, y1, x2, y2);
    }

    vector<int> path; // 记录最优路径的点的一维坐标
    vector<int> fa;
    set<int> pointSet;

    // 随机生成点
    int randPoint(){
        int x = rand() % rows;
        int y = rand() % cols;
        return two2one(x, y);
    }

    // 寻找pointSet中距离point最近的点
    int getNearestPoint(int point, set<int> &pointSet){
        int target = -1;
        double targetDis = 1e9 + 7;
        for(auto p : pointSet){
            double distance = dis(point, p);
            if (distance < targetDis){
                targetDis = distance;
                target = p;
            }
        }
        return target;
    }

    // 步长扩展点
    pair<bool, int> goStep(int curPoint, int dirPoint){
        double distance = dis(curPoint, dirPoint);
        auto [curX, curY] = one2two(curPoint);
        auto [dirX, dirY] = one2two(dirPoint);
        // 比例关系
        int nxtX = curX + 1.0 * (dirX - curX) * stepDistance / distance;
        int nxtY = curY + 1.0 * (dirY - curY) * stepDistance / distance;
        int nxtPoint = two2one(nxtX, nxtY);
        // 扩展点不在障碍物，未出界，连线无障碍，非点集点
        return {valid(nxtX, nxtY) && checkLine(nxtX, nxtY, curX, curY) && pointSet.find(nxtPoint) == pointSet.end(), nxtPoint};
    }

    // 重新选择父亲
    int getBestFatherPoint(int point, set<int>& pointSet, vector<double>& pointDistance){
        int target = 0;
        double targetDis = 1e9 + 7;
        for(auto p : pointSet){ // 枚举点集的点
            double distance = dis(point, p);
            // 距离在方圆内，且从该点到起点距离更小
            if (distance < fatherDistance && checkLine(point, p) && pointDistance[p] + distance < targetDis){
                targetDis = pointDistance[p] + distance;
                target = p;
            }
        }
        return target;
    }

    // 更新距离
    void update(int point, vector<double>& pointDistance){
        for(auto nxtPoint : edge[point]){
            pointDistance[nxtPoint] = dis(point, nxtPoint) + pointDistance[point];
            update(nxtPoint, pointDistance);
        }
    }

    // 重布线
    void rewrite(int point, set<int>& pointSet, vector<double>& pointDistance){
        for(auto p : pointSet){ // 枚举点集的点
            double distance = dis(point, p);
            // 判断距离是否在方圆内，且之间连边无障碍物
            if (distance > rewriteDistance || !checkLine(point, p))
                continue;
            // 如果距离更优
            if (pointDistance[p] > pointDistance[point] + distance){
                // 更新距离
                pointDistance[p] = pointDistance[point] + distance;
                // 删除原先边
                edge[fa[p]].erase(p);
                fa[p] = point;
                // 建立新的边
                edge[point].insert(p);
                // 更新点p之后的点的最短距离
                update(p, pointDistance);
            }
        }
    }

    // 计算到终点距离，判断是否在终点附近
    bool nearTargetPoint(int point){
        return dis(point, two2one(ex, ey)) < targetDistance;
    }

    void findPath(){ 
        fa.resize(rows * cols);
        fill(fa.begin(), fa.end(), 0);
        // 距离起点的数据
        vector<double> distance(rows * cols, 1e9 + 7);
        // 记录从父亲走向儿子的有向边
        edge.resize(rows * cols);
        int st = two2one(sx, sy);
        distance[st] = 0;
        int ed = 0;
        // 已扩展点集
        pointSet.insert(st);
        while(true){
            // 随机采样点
            int samplePoint = randPoint();
            // 在点集中找到距离采样点最近的点
            int nearstPoint = getNearestPoint(samplePoint, pointSet);
            // 以采样点方向以stepDistance步长得到新点，valid表示该点是否合法，即该点是否位于障碍物或超出界，或已在扩展点集里，或沿途有障碍物
            auto [valid, newPoint] = goStep(nearstPoint, samplePoint);
            if (!valid)
                continue;
            // 重新选择父亲点，选一条从st -> fatherPoint -> newPoint最近的fatherPoint
            int fatherPoint = getBestFatherPoint(newPoint, pointSet, distance);
            // 指定父亲
            fa[newPoint] = fatherPoint;
            // 更新距离
            distance[newPoint] = distance[fatherPoint] + dis(newPoint, fatherPoint);
            // 建立新有向边
            edge[fatherPoint].insert(newPoint);
            // 扩展点集合
            pointSet.insert(newPoint);
            // 重布线操作
            rewrite(newPoint, pointSet, distance);
            // 判断是否在终点附近
            if (nearTargetPoint(newPoint)){
                ed = newPoint;
                break;
            }
        }
        // 记录路径
        for(int point = ed; point != st; point = fa[point])
            path.push_back(point);
        path.push_back(st);
        reverse(path.begin(), path.end());
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

    void drawPoint(){
        for(auto u : pointSet){
            auto [y, x] = one2two(u);
            cv::rectangle(maze, cv::Point_<int>(x, y), cv::Point_<int>(x + 1, y + 1), cv::Scalar_<double>(255, 0, 0), -1);
        }
    }

    // 绘制连边

    void drawNei(){
        int st = two2one(sx, sy);
        for(auto u : pointSet){
            if (u == st)
                continue;
            int v = fa[u];
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
