# 简介

>此SDK包仅适用于乐动工程训练营，所用雷达型号如下:
>1. `LDROBOT LiDAR LD06.` 
>2. `LDROBOT LiDAR STL06P.`

- 仓库文件说明

  ```bash
  --./
    - ldlidar_driver/ # 雷达驱动程序
    - linux_app/ # linux系统下的SLAM与路径规划例程,包含封装好的音频播报函数
    - README.md 
  ```
  
- 在使用此SDK前请确保成功安装`OpenCV`、`asound`库,可使用以下命令安装
  ```bash
  sudo apt update
  sudo apt install libopencv-dev
  sudo apt install libasound2-dev
  ```
- 请确保脚本有运行权限,在`linux_app`目录下执行以下命令
  ```bash
  sudo chmod 777 auto_build.sh clean_build.sh start_node.sh
  ```

# 操作指南

## 1. 编译

- 移动到`linux_app`目录
  ```bash
  cd ./linux_app/
  ```

- 然后执行以下命令对文件进行编译
  ```bash
  ./auto_build.sh
  ```

## 2. 运行
- 确保编译成功后，可执行文件生成在`build/`目录下，运行以下脚本，并输入可执行文件的名称即可运行
  ``` bash
  ./start_node.sh
  xxx_demo
  ```

## 3. 清除中间文件和日志
- 执行以下命令
  ```bash
  ./clean_build.sh
  ```