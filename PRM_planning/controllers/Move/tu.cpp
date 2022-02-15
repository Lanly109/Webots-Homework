#include "tu.h"
using namespace std;

int main(){
    srand(time(0));
    tuP maze("./maze.png");
    maze.findPath();
    // maze.randGenPoint();
    // maze.connectNei();
    maze.drawPoint();
    // maze.savePic("example_point.png");
    maze.drawNei();
    // maze.savePic("example_nei.png");
    maze.drawPath();
    // maze.savePic("example_path.png");
    maze.savePath("example.txt");
    return 0;
}
