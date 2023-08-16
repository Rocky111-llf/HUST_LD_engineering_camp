/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD06 products
 * sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ldlidar_driver.h"
#include "opencv2/opencv.hpp"
#include "lidar_data_common.h"
#include "astar.h"
#include "grid_map_2d.h"
#include "rtree.h"

// 获取时间戳
uint64_t GetSystemTimeStamp(void)
{
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp =
        std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
    return ((uint64_t)tmp.count());
}

// 日志打印雷达启动提示
void LidarPowerOn(void)
{
    LDS_LOG_DEBUG("Please Lidar Power On", "");
}

// 日志打印雷达关闭提示
void LidarPowerOff(void)
{
    LDS_LOG_DEBUG("Please Lidar Power Off", "");
}

// 往地图里插入雷达数据帧
int InsertFrameToMap(rbox::RTree2f &map, PointDataFrame insert_frame, float x, float y, float rad)
{
    for (size_t i = 0; i < insert_frame.data.size(); i++)
    {
        PointXY tran_point;
        tran_point.x = insert_frame.data[i].x * cos(rad) + insert_frame.data[i].y * sin(rad) + x;
        tran_point.y = insert_frame.data[i].y * cos(rad) - insert_frame.data[i].x * sin(rad) + y;
        float ins_point[2];
        ins_point[0] = tran_point.x;
        ins_point[1] = tran_point.y;
        map.Insert(&ins_point[0], &ins_point[0], 0);
    }
    return 0;
}

// 评估变换后的点数据在空间上与地图的吻合程度
double CalculatePointDataCostWithRTree(rbox::RTree2f &map, PointDataFrame new_frame, float x, float y, float rad)
{
    double total_value = 0;
    for (size_t i = 0; i < new_frame.data.size(); i++)
    {
        PointXY tran_point;
        tran_point.time_interval=new_frame.data[i].time_interval;
        tran_point.x = new_frame.data[i].x * cos(rad) + new_frame.data[i].y * sin(rad) + x;
        tran_point.y = new_frame.data[i].y * cos(rad) - new_frame.data[i].x * sin(rad) + y;

        float knn_point[2];
        knn_point[0] = tran_point.x;
        knn_point[1] = tran_point.y;

        std::vector<float> dis;
        map.KNN(knn_point, 1, dis);

        total_value += dis[0];
    }
    return total_value;
}

rbox::RTree2f rtree_map;

int main(int argc, char **argv)
{

    // 判断雷达参数是否满足要求，不满足则退出
    if (argc < 4)
    {
        LDS_LOG_WARN("Terminal >>: ./ldlidar_stl <product_name> serialcom <serial_number>", "");
        LDS_LOG_WARN("For example:", "");
        LDS_LOG_WARN("./ldlidar_stl LD06 serialcom /dev/ttyUSB0", "");
        LDS_LOG_WARN("./ldlidar_stl LD19 serialcom /dev/ttyUSB0", "");
        LDS_LOG_WARN("./ldlidar_stl STL06P serialcom /dev/ttyUSB0", "");
        LDS_LOG_WARN("./ldlidar_stl STL27L serialcom /dev/ttyUSB0", "");
        LDS_LOG_WARN("./ldlidar_stl STL26 serialcom /dev/ttyUSB0", "");
        exit(EXIT_FAILURE);
    }

    std::string product_name(argv[1]);
    std::string communication_mode(argv[2]);
    std::string port_name;
    // std::string server_ip;
    // std::string server_port;
    uint32_t serial_baudrate = 0;
    ldlidar::LDType type_name;

    // 默认只有串口模式，不是串口则报错
    if (communication_mode != "serialcom")
    {
        LD_LOG_ERROR("Terminal:input argc(3) value is error", "");
        LDS_LOG_WARN("Terminal >>: ./ldlidar_stl <product_name> serialcom <serial_number>", "");
        LDS_LOG_WARN("For example:", "");
        LDS_LOG_WARN("./ldlidar_stl LD06 serialcom /dev/ttyUSB0", "");
        LDS_LOG_WARN("./ldlidar_stl LD19 serialcom /dev/ttyUSB0", "");
        LDS_LOG_WARN("./ldlidar_stl STL06P serialcom /dev/ttyUSB0", "");
        LDS_LOG_WARN("./ldlidar_stl STL27L serialcom /dev/ttyUSB0", "");
        LDS_LOG_WARN("./ldlidar_stl STL26 serialcom /dev/ttyUSB0", "");
        exit(EXIT_FAILURE);
    }
    else
    {
        if (argc != 4)
        {
            LD_LOG_ERROR("Terminal:input argc != 4", "");
            exit(EXIT_FAILURE);
        }
        port_name = argv[3];
    }

    ldlidar::LDLidarDriver *node = new ldlidar::LDLidarDriver();

    LDS_LOG_INFO("LDLiDAR SDK Pack Version is %s", node->GetLidarSdkVersionNumber().c_str());

    node->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp)); //   注册时间戳获取函数

    node->EnableFilterAlgorithnmProcess(true); // 打开滤波器

    if (product_name == "LD06")
    {
        serial_baudrate = 230400;
        type_name = ldlidar::LDType::LD_06;
    }
    else if (product_name == "STL06P")
    {
        serial_baudrate = 230400;
        type_name = ldlidar::LDType::STL_06P;
    }
    else
    {
        LDS_LOG_ERROR("input <product_name> is error!", "");
        exit(EXIT_FAILURE);
    }

    if (communication_mode == "serialcom")
    {
        if (node->Start(type_name, port_name, serial_baudrate, ldlidar::COMM_SERIAL_MODE))
        {
            LDS_LOG_INFO("ldlidar node start is success", "");
            LidarPowerOn();
        }
        else
        {
            LD_LOG_ERROR("ldlidar node start is fail", "");
            exit(EXIT_FAILURE);
        }
    }
    else
    {
        LDS_LOG_ERROR("input <communication_mode> is error!", "");
        exit(EXIT_FAILURE);
    }

    if (node->WaitLidarCommConnect(3000))
    {
        LDS_LOG_INFO("ldlidar communication is normal.", "");
    }
    else
    {
        LDS_LOG_ERROR("ldlidar communication is abnormal.", "");
        exit(EXIT_FAILURE);
    }

    // 储存雷达数据帧的变量
    ldlidar::Points2D last_frame;
    ldlidar::Points2D new_frame;
    PointDataFrame last_frame_xy;
    PointDataFrame new_frame_xy;

    // 帧数，0->第一帧
    int count = 0;

    // 雷达相对于起始点移动后的坐标
    double car_x = 0;
    double car_y = 0;
    double car_a = 0;

    // 雷达旋转频率
    double lidar_spin_freq;

    // 显示倍率
    int cv_multiple = 500;

    // 上一帧数据时雷达相对于起始点的坐标
    double last_insert_x = 0;
    double last_insert_y = 0;
    double last_insert_a = 0;

    // 栅格图变量
    GridMap2D grid_map(1200, 800, 0.02);
    cv::Mat grid_show(grid_map.get_h(), grid_map.get_w(), CV_8UC1);

    // 点云图变量
    const int show_w = 400;
    const int show_h = 300;
    cv::Mat points_show(show_h, show_w, CV_8UC3);
    memset(&points_show.data[0], 255, show_h * show_w * 3);

    while (ldlidar::LDLidarDriver::IsOk())
    {
        // 每次显示一帧雷达数据前先清空显示区域
        // memset(&points_show.data[0], 255, show_h * show_w * 3);
        LDS_LOG_INFO("count:%d", count);
        LidarDataTransform data_tran_test;

        if (count == 0)
        {
            // 雷达第一帧数据
            switch (node->GetLaserScanData(last_frame, 1500))
            {
            case ldlidar::LidarStatus::NORMAL:
            {
                data_tran_test.set_lidar_data(last_frame);
                // 去除密集点，间隔多少点采样
                // data_tran_test.DataGridFilter(0.1);
                // data_tran_test.DataDownSample(3);
                last_frame_xy = data_tran_test.get_point_data();
                InsertFrameToMap(rtree_map, last_frame_xy, 0, 0, 0);
                count++;
                break;
            }
            case ldlidar::LidarStatus::ERROR:
            {
                LDS_LOG_ERROR("ldlidar is error.", "");
                node->Stop();
                break;
            }
            case ldlidar::LidarStatus::DATA_TIME_OUT:
            {
                LDS_LOG_ERROR("ldlidar publish data is time out, please check your lidar device.", "");
                node->Stop();
                break;
            }
            case ldlidar::LidarStatus::DATA_WAIT:
            {
                LDS_LOG_ERROR("ldlidar is WAIT.", "");
                break;
            }
            default:
            {
                LDS_LOG_ERROR("default", "");
                break;
            }
            }
            usleep(1000 * 100); // sleep 100ms  == 10Hz
            continue;
        }
        // 第二帧之后的数据
        switch (node->GetLaserScanData(new_frame, 1500))
        {
        case ldlidar::LidarStatus::NORMAL:
        {
            // get lidar normal data
            if (node->GetLidarSpinFreq(lidar_spin_freq))
            {
                LDS_LOG_INFO("speed(Hz):%f", lidar_spin_freq);
            }
            data_tran_test.set_lidar_data(new_frame);
            // 去除密集点，间隔多少点采样
            // data_tran_test.DataGridFilter(0.1);
            // data_tran_test.DataDownSample(3);
            new_frame_xy = data_tran_test.get_point_data();

            int step = 1;
            float xy_step = 0.008;
            float ang_step = 0.08;
            float dst_x = 0;
            float dst_y = 0;
            float dst_a = 0;

            double min_value = 36000;

            while (step < 6)
            {
                float new_x = dst_x;
                float new_y = dst_y;
                float new_a = dst_a;
                int direction = 1;
                while (direction < 7)
                {
                    switch (direction)
                    {
                    case 1:
                    {
                        new_x = dst_x + xy_step;
                    }
                    break;
                    case 2:
                    {
                        new_x = dst_x - xy_step;
                    }
                    break;
                    case 3:
                    {
                        new_y = dst_y + xy_step;
                    }
                    break;
                    case 4:
                    {
                        new_y = dst_y - xy_step;
                    }
                    break;
                    case 5:
                    {
                        new_a = dst_a + ang_step;
                    }
                    break;
                    case 6:
                    {
                        new_a = dst_a - ang_step;
                    }
                    break;
                    default:
                        break;
                    }

                    double cost_value = CalculatePointDataCostWithRTree(rtree_map, new_frame_xy, new_x, new_y, new_a);
                    if (cost_value < min_value)
                    {
                        dst_x = new_x;
                        dst_y = new_y;
                        dst_a = new_a;
                        min_value = cost_value;
                        direction = 1;
                    }
                    else
                    {
                        direction++;
                    }
                }
                step++;
                xy_step /= 2;
                ang_step /= 2;
            }

            car_x = dst_x;
            car_y = dst_y;
            car_a = dst_a;

            LDS_LOG_INFO("x=%f,y=%f,angle=%f", car_x, car_y, car_a / 3.1415926 * 180.0);

            grid_map.UpdataMap(new_frame_xy, car_x, car_y, car_a);
            if (((abs(car_x - last_insert_x) + abs(car_y - last_insert_y)) > 0.2) || (abs(car_a - last_insert_a) > 0.5))
            {
                LDS_LOG_DEBUG("count:%d",count);
                InsertFrameToMap(rtree_map, new_frame_xy, car_x, car_y, car_a);
                grid_map.UpdataMap(new_frame_xy, car_x, car_y, car_a);

                last_insert_x = car_x;
                last_insert_y = car_y;
                last_insert_a = car_a;
            }

            int idx_x = car_x * cv_multiple;
            idx_x = idx_x + show_w / 2;
            int idx_y = car_y * cv_multiple;
            idx_y = idx_y + show_h / 2;

            // 绘出雷达移动轨迹
            if ((idx_x < show_w) && (idx_y < show_h) && (idx_x >= 0) && (idx_y >= 0))
            {
                cv::circle(points_show, cv::Point(idx_x, idx_y), 1, cv::Scalar(255, 0, 0));
            }

            // 绘出地图的点云图
            // 大坑,雷达坐标系和opencv坐标系均为左手坐标系
            for (size_t i = 0; i < new_frame_xy.data.size(); i++)
            {
                int idx_x = (new_frame_xy.data[i].x * cos(car_a) + new_frame_xy.data[i].y * sin(car_a) + car_x) * cv_multiple;
                idx_x = idx_x + show_w / 2;
                int idx_y = (new_frame_xy.data[i].y * cos(car_a) - new_frame_xy.data[i].x * sin(car_a) + car_y) * cv_multiple;
                idx_y = idx_y + show_h / 2;
                if ((idx_x < show_w) && (idx_y < show_h) && (idx_x >= 0) && (idx_y >= 0))
                {
                    cv::circle(points_show, cv::Point(idx_x, idx_y), 0, cv::Scalar(0, 0, 255));
                }
            }

            // 将栅格图用opencv显示
            LDS_LOG_DEBUG("grid_map_w:%d,grid_map_h:%d",grid_map.get_w(),grid_map.get_h());
            memcpy(&grid_show.data[0], grid_map.get_map(), grid_map.get_w() * grid_map.get_h());
            cv::flip(grid_show, grid_show, 0);

            cv::imshow("grid_map", grid_show);
            cv::imshow("trojectory", points_show);
            cv::waitKey(1);

            break;
        }
        case ldlidar::LidarStatus::ERROR:
        {
            LDS_LOG_ERROR("ldlidar is error.", "");
            node->Stop();
            break;
        }
        case ldlidar::LidarStatus::DATA_TIME_OUT:
        {
            LDS_LOG_ERROR("ldlidar publish data is time out, please check your lidar device.", "");
            node->Stop();
            break;
        }
        case ldlidar::LidarStatus::DATA_WAIT:
        {
            break;
        }
        default:
        {
            break;
        }
        }

        last_frame_xy = new_frame_xy;
        count++;
        // 不延时
        usleep(1000 * 50); // sleep 100ms  == 10Hz
    }

    node->Stop();
    LidarPowerOff();

    delete node;
    node = nullptr;

    return 0;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/