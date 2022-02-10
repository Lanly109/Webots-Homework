#include "tu.h"
// #include <signal.h>
#include <stdlib.h>
#include <stdio.h>
// #include <unistd.h>
using namespace std;

tuP maze("./maze.png");

// void myhandle(int s){
//     printf("Catch exception, interupting, saveing and exiting...\n");
//     maze.drawPoint();
//     maze.savePic("qwq_point.png");
//     maze.drawNei();
//     maze.savePic("qwq_line.png");
//     exit(0);
// }

int main(){
    // struct sigaction sigIntHandler;
    //
    // sigIntHandler.sa_handler = myhandle;
    // sigemptyset(&sigIntHandler.sa_mask);
    // sigIntHandler.sa_flags = 0;
    //
    // sigaction(SIGINT, &sigIntHandler, NULL);
    srand(time(0));
    maze.findPath();
    // maze.randGenPoint();
    // maze.connectNei();
    maze.drawPoint();
    maze.savePic("example_point.png");
    maze.drawNei();
    maze.savePic("example_nei.png");
    maze.drawPath();
    maze.savePic("example_path.png");
    maze.savePath("example.txt");
    return 0;
}
