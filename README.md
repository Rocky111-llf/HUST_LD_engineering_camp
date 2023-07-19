# 简介

>此SDK包仅适用于乐动工程训练营，所用雷达型号如下:
>1. `LDROBOT LiDAR LD06.` 
>2. `LDROBOT LiDAR STL06P.`

- 仓库文件说明

  ```bash
  --./
    - ldlidar_driver/ # 雷达驱动程序
    - linux_app/ # linux系统下的雷达数据获取与点云显示例程
    - README.md 
  ```
  
- 在使用此SDK前请确保以成功安装`OpenCV`,可使用以下命令安装
  ```bash
  sudo apt update
  sudo apt install libopencv-dev
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
- 确保编译成功后，执行以下命令，即可得到雷达数据和点云图
  ``` bash
  ./start_node.sh
  ```

## 3. 清除中间文件和日志
- 执行以下命令
  ```bash
  ./clean_build.sh
  ```