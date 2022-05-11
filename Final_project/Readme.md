# Webots Final Project

演示视频：[bilibili](https://www.bilibili.com/video/BV1pm4y1Z7ee)

实验平台：Webots 2021a

注：2022a版对坐标系进行了改变，详情点击[此处](https://github.com/cyberbotics/webots/wiki/How-to-adapt-your-world-or-PROTO-to-Webots-R2022a)，故请使用此版本之前的平台，比如2021a。当然你也可以使用该网站提到的的[脚本](https://github.com/cyberbotics/webots/blob/master/scripts/converter/convert_nue_to_enu_rub_to_flu.py)对世界文件进行坐标变换

小车通过感知周围环境，绕开障碍物抵达终点。

其中小车位置和终点位置通过`GPS`定位获得。

感知参考`SLAM`写法。

规划采用`D*`算法。

运动使用了`PID`控制器。

```bash
.
├── Readme.md           // 须知
├── controllers         // 控制器
│   ├── openDoor        // 门开关控制逻辑（world3里）
│   │   ├── Makefile    
│   │   └── openDoor.c
│   └── wasd            // 小车控制逻辑
│       ├── Makefile    // 开了C++17
│       └── wasd.cpp
└── worlds
    ├── world1.wbt      // 迷宫一
    ├── world2.wbt      // 迷宫二
    ├── world3.wbt      // 迷宫三
    └── world4.wbt      // 迷宫四
```

双击`world`文件夹下的`wbt`文件打开世界。

### 控制器代码

务必开启 `c++17`标准。

由于不同世界的尺寸不一样，需要更改`wasd.cpp`的`22-40`行的注释，以适应不同世界的尺寸，使得在感知出的图的障碍物定位与实际一致。
