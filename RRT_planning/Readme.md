# Webots RTT*路径规划

实验平台：Webots 2021a

注：2022a版对坐标系进行了改变，详情点击[此处](https://github.com/cyberbotics/webots/wiki/How-to-adapt-your-world-or-PROTO-to-Webots-R2022a)，故请使用此版本之前的平台，比如2021a。当然你也可以使用该网站提到的的[脚本](https://github.com/cyberbotics/webots/blob/master/scripts/converter/convert_nue_to_enu_rub_to_flu.py)对世界文件进行坐标变换

```bash
.
├── controllers
│   ├── Move                                // 运动规划控制器
│   │   ├── Makefile                        // 开了C++17
│   │   ├── Move.cpp                        // 控制器代码
│   │   ├── example.png                     // 规划路径一的图
│   │   ├── example.txt                     // 保存了规划路径一的数据
│   │   ├── maze.png                        // 原图
│   │   ├── example_path_far.png            // 规划路径二的图
│   │   ├── example_path_far.txt            // 保存了规划路径二的数据
│   │   ├── example_path_norewrite.png      // 规划路径三的图
│   │   ├── example_path_norewrite.txt      // 保存了规划路径三的数据
│   │   ├── test.cpp                        // 源代码，能键盘控制小车上下左右移动
│   │   ├── tu.cpp                          // RRT算法主程序
│   │   └── tu.h                            // 实现RRT算法的类
│   └── display
│       ├── Makefile                        // 开了C++17
│       └── display.cpp                     // 可视化控制器代码
├── libraries
├── maze.png
├── plugins
├── protos
├── Readme.md                               // 本须知
└── worlds                                  // 世界文件
```

双击`world/empty.wbt`打开世界。

### 编译
编译 `tu.cpp`
```bash
g++ tu.cpp `pkg-config --cflags --libs opencv` -std=c++17 -g -o tu  # 部分电脑可能是opencv4
./tu
``` 

如非 `bash`环境或未安装`pkg-config`或未对opencv配置，请自行更改为将``` `pkg-config --cflags --libs opencv` ```更改为链接本机`OpenCV`库的参数指令。

如在Mac下``` `pkg-config --cflags --libs opencv` ```的运行结果为
```bash
-I/opt/homebrew/opt/opencv/include/opencv4 -L/opt/homebrew/opt/opencv/lib -lopencv_gapi -lopencv_stitching -lopencv_alphamat -lopencv_aruco -lopencv_barcode -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dnn_objdetect -lopencv_dnn_superres -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hfs -lopencv_img_hash -lopencv_intensity_transform -lopencv_line_descriptor -lopencv_mcc -lopencv_quality -lopencv_rapid -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_sfm -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_superres -lopencv_optflow -lopencv_surface_matching -lopencv_tracking -lopencv_highgui -lopencv_datasets -lopencv_text -lopencv_plot -lopencv_videostab -lopencv_videoio -lopencv_viz -lopencv_wechat_qrcode -lopencv_xfeatures2d -lopencv_shape -lopencv_ml -lopencv_ximgproc -lopencv_video -lopencv_dnn -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_imgcodecs -lopencv_features2d -lopencv_flann -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core
``` 

通过修改`tu.h`文件下的 `REC, stepDistance, rewriteDistance, fatherDistance, targetDistance`变量可以得到不同的路径图。部分情况可能求解时间超过1s，还请耐心等待。

**限于系统原因，OpenCV安装时所编译的动态链接库为Arm架构，而Webots暂未对MacOS的M1类型适配，其动态链接库均为x86_64架构。这将导致Webots在生成项目时，在链接动态链接库阶段出现找不到OpenCV相关成员变量的x86_64的动态库，而在手动用命令行编译时，亦出现找不到Webots相关成员变量的Arm的动态库。因此这里将这两个代码分开，即路径规划和巡线代码为不同文件，其中路径规划代码引用了OpenCV库，在命令行编译下能够正常编译成功运行，巡线代码在Webots下亦能正常编译成功运行。而理论上它们在Linux或Windows应能融洽在同一个代码中。**

### 控制器代码

务必开启 `c++17`标准。

通过修改109行的`Path maze("./example.txt");`中的文件名来实现不同的路径寻路。

同时也要相应的修改 `display.cpp`文件的118行的`Path maze("../Move/example.txt");`于相应的文件。
