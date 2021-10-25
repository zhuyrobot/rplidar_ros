/*
 *  RPLIDAR ROS2 NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include "rplidar.h"

#include <signal.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

#define ROS2VERSION "1.0.1"

using namespace std;
using namespace rp::standalone::rplidar;

bool need_exit = false;

class RPLidarScanPublisher : public rclcpp::Node
{
public:
	RPLidarScanPublisher() : Node("rplidar_scan_publisher") {}

private:
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::SensorDataQoS()));
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_service = this->create_service<std_srvs::srv::Empty>("stop_motor",
		[this](const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)->void
		{
			if (!drv) { RCLCPP_ERROR(this->get_logger(), "Failed: Not create driver"); return; }
			if (!drv->isConnected()) { RCLCPP_ERROR(this->get_logger(), "Failed: Not connect device"); return; }
			u_result ret = drv->stopMotor();
			if (IS_FAIL(ret)) RCLCPP_ERROR(this->get_logger(), "Failed: stopMotor and ret=%x", ret);
			else RCLCPP_INFO(this->get_logger(), "Done: stopMotor");
		});
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_service = this->create_service<std_srvs::srv::Empty>("start_motor",
		[this](const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
		{
			if (!drv) { RCLCPP_ERROR(this->get_logger(), "Failed: Not create driver"); return; }
			if (!drv->isConnected()) { RCLCPP_ERROR(this->get_logger(), "Failed: Not connect device"); return; }
			u_result ret = drv->startMotor();
			if (IS_FAIL(ret)) { RCLCPP_WARN(this->get_logger(), "Failed: startMotor and ret=%x", ret); return; }
			else RCLCPP_INFO(this->get_logger(), "Done: startMotor");
			ret = drv->startScan(false, true);
			if (IS_FAIL(ret)) RCLCPP_WARN(this->get_logger(), "Failed: startScan and ret=%x", ret);
			else RCLCPP_INFO(this->get_logger(), "Done: startScan");
		});

	string channel_type;
	string tcp_ip;
	string serial_port;
	int tcp_port = 20108;
	int serial_baudrate = 115200;
	string frame_id = "laser_frame";
	bool inverted = false;
	bool angle_compensate = true;
	float max_distance = 8.0;//not be declared.
	size_t angle_compensate_multiple = 1;//not be declared. it stand of angle compensate at per 1 degree
	string scan_mode;

private:
	static float getAngle(const rplidar_response_measurement_node_hq_t& node) { return node.angle_z_q14 * 90.f / 16384.f; }
	void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub,
		rplidar_response_measurement_node_hq_t* nodes,
		size_t node_count, rclcpp::Time start,
		double scan_time, bool inverted,
		float angle_min, float angle_max,
		float max_distance, string frame_id)
	{
		static int scan_count = 0;
		auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

		scan_msg->header.stamp = start;
		scan_msg->header.frame_id = frame_id;
		scan_count++;

		bool reversed = (angle_max > angle_min);
		if (reversed)
		{
			scan_msg->angle_min = M_PI - angle_max;
			scan_msg->angle_max = M_PI - angle_min;
		}
		else
		{
			scan_msg->angle_min = M_PI - angle_min;
			scan_msg->angle_max = M_PI - angle_max;
		}
		scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (double)(node_count - 1);

		scan_msg->scan_time = scan_time;
		scan_msg->time_increment = scan_time / (double)(node_count - 1);
		scan_msg->range_min = 0.15;
		scan_msg->range_max = max_distance;//8.0;

		scan_msg->intensities.resize(node_count);
		scan_msg->ranges.resize(node_count);
		bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
		if (!reverse_data)
		{
			for (size_t i = 0; i < node_count; i++)
			{
				float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
				if (read_value == 0.0) scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
				else scan_msg->ranges[i] = read_value;
				scan_msg->intensities[i] = (float)(nodes[i].quality >> 2);
			}
		}
		else
		{
			for (size_t i = 0; i < node_count; i++)
			{
				float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
				if (read_value == 0.0) scan_msg->ranges[node_count - 1 - i] = std::numeric_limits<float>::infinity();
				else scan_msg->ranges[node_count - 1 - i] = read_value;
				scan_msg->intensities[node_count - 1 - i] = (float)(nodes[i].quality >> 2);
			}
		}
		pub->publish(*scan_msg);
	}

public:
	RPlidarDriver* drv;
	int work_loop()
	{
		//0.InitParam
		{
			this->declare_parameter("channel_type");
			this->declare_parameter("tcp_ip");
			this->declare_parameter("tcp_port");
			this->declare_parameter("serial_port");
			this->declare_parameter("serial_baudrate");
			this->declare_parameter("frame_id");
			this->declare_parameter("inverted");
			this->declare_parameter("angle_compensate");
			this->declare_parameter("scan_mode");

			this->get_parameter_or<string>("channel_type", channel_type, "serial");
			this->get_parameter_or<string>("tcp_ip", tcp_ip, "192.168.0.7");
			this->get_parameter_or<int>("tcp_port", tcp_port, 20108);
			this->get_parameter_or<string>("serial_port", serial_port, "/dev/ttyUSB0");
			this->get_parameter_or<int>("serial_baudrate", serial_baudrate, 115200/*256000*/);//ros run for A1 A2, change to 256000 if A3
			this->get_parameter_or<string>("frame_id", frame_id, "laser_frame");
			this->get_parameter_or<bool>("inverted", inverted, false);
			this->get_parameter_or<bool>("angle_compensate", angle_compensate, false);
			this->get_parameter_or<string>("scan_mode", scan_mode, string());
		}
		RCLCPP_INFO(this->get_logger(), "ROS2 SDK Version:" ROS2VERSION ", RPLIDAR SDK Version:" RPLIDAR_SDK_VERSION "");

		//1.CreateDriver
		if (channel_type == "tcp") drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_TCP);
		else drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
		if (!drv) { RCLCPP_ERROR(this->get_logger(), "Failed: create driver"); return -2; }
		else RCLCPP_INFO(this->get_logger(), "Done: create driver");

		//2.ConnectLidar
		u_result ret;
		if (channel_type == "tcp") ret = drv->connect(tcp_ip.c_str(), uint32_t(tcp_port));
		else ret = drv->connect(serial_port.c_str(), uint32_t(serial_baudrate));
		char connstr[127]; sprintf(connstr, "connect %s with %d", tcp_ip.c_str(), tcp_port);
		if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: %s and ret=%d", connstr, ret); RPlidarDriver::DisposeDriver(drv); return -1; }
		else RCLCPP_INFO(this->get_logger(), "Done: %s", connstr);

		//3.GetDeviceInfo
		rplidar_response_device_info_t devinfo;
		ret = drv->getDeviceInfo(devinfo);
		if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: getDeviceInfo and ret=%x", ret); RPlidarDriver::DisposeDriver(drv); return -1; }
		else
		{
			string sn_str;
			for (int pos = 0; pos < 16; ++pos)
			{
				char sn[3] = {};
				sprintf(sn, "%02X", devinfo.serialnum[pos]);
				sn_str += string(sn, sn + 2);
			}
			RCLCPP_INFO(this->get_logger(), "RPLidarSN: %s", sn_str.c_str());
			RCLCPP_INFO(this->get_logger(), "FirmwareVer: %d.%02d", devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF);
			RCLCPP_INFO(this->get_logger(), "HardwareRev: %d", int(devinfo.hardware_version));
		}

		//4.CheckHealth
		rplidar_response_device_health_t healthinfo;
		ret = drv->getHealth(healthinfo);
		if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: getHealth and ret=%x", ret); RPlidarDriver::DisposeDriver(drv); return -1; }
		else RCLCPP_INFO(this->get_logger(), "RPLidarHealthStatus : %d", healthinfo.status);
		return true;

		//5.StartMotor
		ret = drv->startMotor();
		if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: startMotor"); RPlidarDriver::DisposeDriver(drv); return -1; }
		else RCLCPP_INFO(this->get_logger(), "Done: startMotor");

		//6.StartScan
		RplidarScanMode current_scan_mode;
		if (scan_mode.empty()) ret = drv->startScan(false, true, 0, &current_scan_mode);
		else//Custom mode
		{
			vector<RplidarScanMode> allSupportedScanModes;
			ret = drv->getAllSupportedScanModes(allSupportedScanModes);
			if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: getAllSupportedScanModes and ret=%x", ret); RPlidarDriver::DisposeDriver(drv); return -1; }
			else
			{
				uint16_t selectedScanMode = -1;
				for (int k = 0; k < allSupportedScanModes.size(); ++k)
					if (allSupportedScanModes[k].scan_mode == scan_mode) { selectedScanMode = allSupportedScanModes[k].id; break; }
				if (selectedScanMode == -1)
				{
					RCLCPP_ERROR(this->get_logger(), "Failed: scan mode %s is not supported, following are supported modes", scan_mode.c_str());
					for (int k = 0; k < allSupportedScanModes.size(); ++k)
						RCLCPP_ERROR(this->get_logger(), "\t%s: max_distance: %.1f m, Point number: %.1fK", 
							allSupportedScanModes[k].scan_mode,
							allSupportedScanModes[k].max_distance, 
							(1000 / allSupportedScanModes[k].us_per_sample));
					RPlidarDriver::DisposeDriver(drv);
					return -1;
				}
				else ret = drv->startScanExpress(false, selectedScanMode, 0, &current_scan_mode);
			}
		}
		if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: startScan/startScanExpress and ret=%x", ret); RPlidarDriver::DisposeDriver(drv); return -1; }
		else
		{
			//default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
			max_distance = current_scan_mode.max_distance;
			angle_compensate_multiple = (int)(1000 * 1000 / current_scan_mode.us_per_sample / 10.0 / 360.0);
			if (angle_compensate_multiple < 1) angle_compensate_multiple = 1;
			RCLCPP_INFO(this->get_logger(), "CurrentScanMode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %d", 
				current_scan_mode.scan_mode,
				current_scan_mode.max_distance, 
				(1000 / current_scan_mode.us_per_sample),
				angle_compensate_multiple);
		}

		//7.
		while (rclcpp::ok() && !need_exit)
		{
			//7.1 GrabScan
			size_t count = 360 * 8;
			rplidar_response_measurement_node_hq_t nodes[360 * 8];
			rclcpp::Time start_scan_time = this->now();
			ret = drv->grabScanDataHq(nodes, count);
			rclcpp::Time end_scan_time = this->now();
			double scan_duration = (end_scan_time - start_scan_time).seconds();
			if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: grabScanDataHq and ret=%x", ret); continue; }

			//7.2 SortScan
			ret = drv->ascendScanData(nodes, count);
			if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: ascendScanData and ret=%x", ret); continue; }
			
			//7.3
			if (!angle_compensate)
			{
				int first_node = -1;
				int final_node = count;
				while (nodes[++first_node].dist_mm_q2 == 0);//First valid node
				while (nodes[--final_node].dist_mm_q2 == 0);//Final valid node
				float angle_min = DEG2RAD(getAngle(nodes[first_node]));
				float angle_max = DEG2RAD(getAngle(nodes[final_node]));

				publish_scan(scan_pub, &nodes[first_node], final_node - first_node + 1, start_scan_time, scan_duration, inverted, angle_min, angle_max, max_distance, frame_id);
			}
			else 
			{
				const int angle_compensate_nodes_count = 360 * angle_compensate_multiple;
				vector<rplidar_response_measurement_node_hq_t> angle_compensate_nodes(angle_compensate_nodes_count);
				memset(angle_compensate_nodes.data(), 0, angle_compensate_nodes_count * sizeof(rplidar_response_measurement_node_hq_t));

				for (size_t i = 0, angle_compensate_offset = 0; i < count; ++i)
					if (nodes[i].dist_mm_q2 != 0)
					{
						float angle = getAngle(nodes[i]);
						int angle_value = int(angle * angle_compensate_multiple);
						if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
						for (size_t j = 0; j < angle_compensate_multiple; ++j)
						{
							int angle_compensate_nodes_index = angle_value - angle_compensate_offset + j;
							if (angle_compensate_nodes_index >= angle_compensate_nodes_count) angle_compensate_nodes_index = angle_compensate_nodes_count - 1;
							angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
						}
					}

				publish_scan(scan_pub, angle_compensate_nodes.data(), angle_compensate_nodes.size(), start_scan_time, scan_duration, inverted, 0.f, 359.f, max_distance, frame_id);
			}

			rclcpp::spin_some(shared_from_this());
		}

		//8.StopAndClean
		if(IS_FAIL(drv->stop())) RCLCPP_ERROR(this->get_logger(), "Failed: stopScan");
		else RCLCPP_INFO(this->get_logger(), "Done: stopScan");
		if (IS_FAIL(drv->stopMotor())) RCLCPP_ERROR(this->get_logger(), "Failed: stopMotor");
		RCLCPP_INFO(this->get_logger(), "Done: stopMotor");
		RPlidarDriver::DisposeDriver(drv);
		return 0;
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto rplidar_scan_publisher = std::make_shared<RPLidarScanPublisher>();
	signal(SIGINT, [](int sig) { need_exit = true; });
	int ret = rplidar_scan_publisher->work_loop();
	rclcpp::shutdown();
	return ret;
}

