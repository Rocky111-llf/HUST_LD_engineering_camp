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

	ldlidar::Points2D laser_scan_points;
	double lidar_spin_freq;
	const int show_w = 400;
	const int show_h = 300;
	cv::Mat points_show(show_h, show_w, CV_8UC3);
	while (ldlidar::LDLidarDriver::IsOk())
	{
		// 每次显示一帧雷达数据前先清空显示区域
		memset(&points_show.data[0], 255, show_h * show_w * 3);
		switch (node->GetLaserScanData(laser_scan_points, 1500))
		{
		case ldlidar::LidarStatus::NORMAL:
			// get lidar normal data
			if (node->GetLidarSpinFreq(lidar_spin_freq))
			{
				LDS_LOG_INFO("speed(Hz):%f", lidar_spin_freq);
			}

#ifdef __LP64__
			LDS_LOG_INFO("size:%d,stamp_front:%lu, stamp_back:%lu",
						 laser_scan_points.size(), laser_scan_points.front().stamp, laser_scan_points.back().stamp);
#else
			LDS_LOG_INFO("size:%d,stamp_front:%llu, stamp_back:%llu",
						 laser_scan_points.size(), laser_scan_points.front().stamp, laser_scan_points.back().stamp);
#endif
			for (auto point : laser_scan_points)
			{
#ifdef __LP64__
				LDS_LOG_INFO("stamp:%lu,angle:%f,distance(mm):%d,intensity:%d",
							 point.stamp, point.angle, point.distance, point.intensity);
#else
				LDS_LOG_INFO("stamp:%llu,angle:%f,distance(mm):%d,intensity:%d",
							 point.stamp, point.angle, point.distance, point.intensity);
#endif
				int idx_x = point.distance * cos(point.angle / 180.0 * 3.14159) / 20;
				idx_x = idx_x + show_w / 2;
				int idx_y = point.distance * sin(point.angle / 180.0 * 3.14159) / 20 + show_h / 2;

				if ((idx_x < show_w) && (idx_y < show_h) && (idx_x >= 0) && (idx_y >= 0))
				{
					cv::circle(points_show, cv::Point(idx_x, idx_y), 0, cv::Scalar(0, 0, 255));
				}
			}
			cv::imshow("lidar frame", points_show);
			cv::waitKey(1);
			break;
		case ldlidar::LidarStatus::ERROR:
			LDS_LOG_ERROR("ldlidar is error.", "");
			node->Stop();
			break;
		case ldlidar::LidarStatus::DATA_TIME_OUT:
			LDS_LOG_ERROR("ldlidar publish data is time out, please check your lidar device.", "");
			node->Stop();
			break;
		case ldlidar::LidarStatus::DATA_WAIT:
			break;
		default:
			break;
		}

		usleep(1000 * 100); // sleep 100ms  == 10Hz
	}

	node->Stop();
	LidarPowerOff();

	delete node;
	node = nullptr;

	return 0;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/