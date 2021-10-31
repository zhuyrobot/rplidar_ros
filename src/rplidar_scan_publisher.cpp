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
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::SensorDataQoS()));
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_service = this->create_service<std_srvs::srv::Empty>("stop_motor",
		[this](const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)->void
		{
			if (!drv) { RCLCPP_ERROR(this->get_logger(), "Failed: Not create driver"); return; }
			if (!drv->isConnected()) { RCLCPP_ERROR(this->get_logger(), "Failed: Not connect device"); return; }
			if (IS_FAIL(ret = drv->stopMotor())) RCLCPP_ERROR(this->get_logger(), "Failed: stopMotor and ret=%x", ret);
			else RCLCPP_INFO(this->get_logger(), "Done: stopMotor");
		});
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_service = this->create_service<std_srvs::srv::Empty>("start_motor",
		[this](const std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
		{
			if (!drv) { RCLCPP_ERROR(this->get_logger(), "Failed: Not create driver"); return; }
			if (!drv->isConnected()) { RCLCPP_ERROR(this->get_logger(), "Failed: Not connect device"); return; }
			if (IS_FAIL(ret = drv->startMotor())) { RCLCPP_ERROR(this->get_logger(), "Failed: startMotor and ret=%x", ret); return; }
			else RCLCPP_INFO(this->get_logger(), "Done: startMotor");
			if (IS_FAIL(ret = drv->startScan(false, true))) RCLCPP_ERROR(this->get_logger(), "Failed: startScan and ret=%x", ret);
			else RCLCPP_INFO(this->get_logger(), "Done: startScan");
		});

	string channel_type = this->declare_parameter("channel_type", "serial");
	string serial_port = this->declare_parameter("serial_port", "/dev/ttyUSB0");
	int serial_baudrate = this->declare_parameter("serial_baudrate", 115200/*256000*/);//ros run for A1 A2, change to 256000 if A3
	string tcp_ip = this->declare_parameter("tcp_ip", "192.168.0.7");
	int tcp_port = this->declare_parameter("tcp_port", 20108);
	string frame_id = this->declare_parameter("frame_id", "laser");
	string scan_mode = this->declare_parameter("scan_mode", "");
	bool inverted = this->declare_parameter("inverted", false);
	bool angle_compensate = this->declare_parameter("angle_compensate", true);
	float max_distance = 8.0;
	size_t angle_compensate_multiple = 1;//it stand of angle compensate at per 1 degree
	u_result ret;
	RPlidarDriver* drv;

private:
	static float getAngle(const rplidar_response_measurement_node_hq_t& node) { return node.angle_z_q14 * 90.f / 16384.f; }
	void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub,
		rplidar_response_measurement_node_hq_t* meas_nodes, size_t node_count,
		rclcpp::Time start_time, double scan_time, bool inverted,
		float angle_min, float angle_max,
		float range_max, string frame_id)
	{
		//0.
		static int scan_count = 0; ++scan_count;
		auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

		//1.
		scan_msg->header.stamp = start_time;
		scan_msg->header.frame_id = frame_id;
		scan_msg->scan_time = scan_time;
		scan_msg->time_increment = scan_time / (node_count - 1);

		//2.
		bool reversed = (angle_max > angle_min);
		scan_msg->angle_min = M_PI - (reversed ? angle_max : angle_min);
		scan_msg->angle_max = M_PI - (reversed ? angle_min : angle_max);
		scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (node_count - 1);

		//3.
		scan_msg->range_min = 0.15;
		scan_msg->range_max = range_max;
		scan_msg->ranges.resize(node_count);
		scan_msg->intensities.resize(node_count);
		bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
		for (size_t i = 0; i < node_count; ++i)
		{
			float distance = float(meas_nodes[i].dist_mm_q2 * 0.00025);///4.0/1000;
			int index = reverse_data ? node_count - 1 - i : i;
			scan_msg->ranges[index] = distance != 0 ? distance : numeric_limits<float>::infinity();
			scan_msg->intensities[index] = float(meas_nodes[i].quality >> 2);
		}
		pub->publish(*scan_msg);
	}

public:
	int work_loop()
	{
		//1.CreateDriver
		RCLCPP_INFO(this->get_logger(), "ROS2 SDK Version:" ROS2VERSION ", RPLIDAR SDK Version:" RPLIDAR_SDK_VERSION "");
		if (channel_type == "tcp") drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_TCP);
		else drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
		if (!drv) { RCLCPP_ERROR(this->get_logger(), "Failed: create driver"); return -2; }
		else RCLCPP_INFO(this->get_logger(), "Done: create driver");

		//2.ConnectLidar
		if (channel_type == "tcp") ret = drv->connect(tcp_ip.c_str(), uint32_t(tcp_port));
		else ret = drv->connect(serial_port.c_str(), uint32_t(serial_baudrate));
		char connstr[127]; sprintf(connstr, "connect %s with %d", (channel_type == "tcp" ? tcp_ip : serial_port).c_str(), channel_type == "tcp" ? tcp_port : serial_baudrate);
		if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: %s and ret=%x", connstr, ret); RPlidarDriver::DisposeDriver(drv); return -1; }
		else RCLCPP_INFO(this->get_logger(), "Done: %s", connstr);

		//3.GetDeviceInfo
		rplidar_response_device_info_t device_info;
		if (IS_FAIL(ret = drv->getDeviceInfo(device_info))) { RCLCPP_ERROR(this->get_logger(), "Failed: getDeviceInfo and ret=%x", ret); RPlidarDriver::DisposeDriver(drv); return -1; }
		else
		{
			string sn_str;
			for (int pos = 0; pos < 16; ++pos)
			{
				char sn[3] = {};
				sprintf(sn, "%02X", device_info.serialnum[pos]);
				sn_str += string(sn, sn + 2);
			}
			RCLCPP_INFO(this->get_logger(), "RPLidarSN: %s", sn_str);
			RCLCPP_INFO(this->get_logger(), "FirmwareVer: %d.%02d", device_info.firmware_version >> 8, device_info.firmware_version & 0xFF);
			RCLCPP_INFO(this->get_logger(), "HardwareRev: %d", int(device_info.hardware_version));
		}

		//4.CheckHealth
		rplidar_response_device_health_t device_health;
		if (IS_FAIL(ret = drv->getHealth(device_health))) { RCLCPP_ERROR(this->get_logger(), "Failed: getHealth and ret=%x", ret); RPlidarDriver::DisposeDriver(drv); return -1; }
		else RCLCPP_INFO(this->get_logger(), "RPLidarHealthStatus : %d", device_health.status);

		//5.StartMotor
		if (IS_FAIL(ret = drv->startMotor())) { RCLCPP_ERROR(this->get_logger(), "Failed: startMotor"); RPlidarDriver::DisposeDriver(drv); return -1; }
		else RCLCPP_INFO(this->get_logger(), "Done: startMotor");

		//6.StartScan
		RplidarScanMode acutal_scan_mode;
		if (scan_mode.empty()) ret = drv->startScan(false, true, 0, &acutal_scan_mode);
		else//Custom mode
		{
			vector<RplidarScanMode> all_scan_modes;
			ret = drv->getAllSupportedScanModes(all_scan_modes);
			if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: getAllSupportedScanModes and ret=%x", ret); RPlidarDriver::DisposeDriver(drv); return -1; }
			else
			{
				uint16_t scan_id = -1;
				for (int k = 0; k < all_scan_modes.size(); ++k) if (all_scan_modes[k].scan_mode == scan_mode) { scan_id = all_scan_modes[k].id; break; }
				if (scan_id == -1)
				{
					RCLCPP_ERROR(this->get_logger(), "Failed: scan mode %s is not supported, following are supported modes", scan_mode);
					for (int k = 0; k < all_scan_modes.size(); ++k)
						RCLCPP_ERROR(this->get_logger(), "ScanMode: %s(max_distance=%.1fm PointNumber=%.1fK)",
							all_scan_modes[k].scan_mode, all_scan_modes[k].max_distance,
							(1000 / all_scan_modes[k].us_per_sample));
					drv->stopMotor();
					RPlidarDriver::DisposeDriver(drv);
					return -1;
				}
				else ret = drv->startScanExpress(false, scan_id, 0, &acutal_scan_mode);
			}
		}
		if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: startScan/startScanExpress and ret=%x", ret); drv->stopMotor(); RPlidarDriver::DisposeDriver(drv); return -1; }
		else
		{
			//default frequent is 10 hz (by motor pwm value),  acutal_scan_mode.us_per_sample is the number of scan point per us
			angle_compensate_multiple = int(1000 * 1000 / acutal_scan_mode.us_per_sample / 10.0 / 360.0);
			if (angle_compensate_multiple < 1) angle_compensate_multiple = 1;
			RCLCPP_INFO(this->get_logger(), "ScanMode: %s(max_distance=%.1fm PointNumber=%.1fK AngleCompensate=%d)",
				acutal_scan_mode.scan_mode, acutal_scan_mode.max_distance,
				(1000 / acutal_scan_mode.us_per_sample), angle_compensate_multiple);
		}

		//7.SpinROS
		while (rclcpp::ok() && !need_exit)
		{
			rclcpp::spin_some(shared_from_this());

			//7.1 GrabScan
			size_t meas_count = 360 * 8;
			vector<rplidar_response_measurement_node_hq_t> meas_nodes(meas_count);
			rclcpp::Time start_scan_time = this->now();
			ret = drv->grabScanDataHq(meas_nodes.data(), meas_count);
			rclcpp::Time end_scan_time = this->now();
			double scan_duration = (end_scan_time - start_scan_time).seconds();
			if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: grabScanDataHq and ret=%x", ret); continue; }

			//7.2 SortScan
			ret = drv->ascendScanData(meas_nodes.data(), meas_count);
			if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: ascendScanData and ret=%x", ret); continue; }

			//7.3 RawScan
			if (!angle_compensate)
			{
				int first_node = -1;
				int final_node = meas_count;
				while (meas_nodes[++first_node].dist_mm_q2 == 0);//First valid node
				while (meas_nodes[--final_node].dist_mm_q2 == 0);//Final valid node
				float angle_min = DEG2RAD(getAngle(meas_nodes[first_node]));
				float angle_max = DEG2RAD(getAngle(meas_nodes[final_node]));

				publish_scan(scan_pub, meas_nodes.data() + first_node, final_node - first_node + 1, start_scan_time, scan_duration, inverted, angle_min, angle_max, max_distance, frame_id);
			}

			//7.4 CompensateScan
			else
			{
				vector<rplidar_response_measurement_node_hq_t> meas_nodes_with_compensation(360 * angle_compensate_multiple);
				memset(meas_nodes_with_compensation.data(), 0, meas_nodes_with_compensation.size() * sizeof(meas_nodes_with_compensation[0]));

				for (size_t i = 0, offset = 0; i < meas_count; ++i)
					if (meas_nodes[i].dist_mm_q2 != 0)
					{
						int angle_value = int(getAngle(meas_nodes[i]) * angle_compensate_multiple);
						if (offset > angle_value) offset = angle_value;
						for (size_t j = 0; j < angle_compensate_multiple; ++j)
						{
							int angle_compensate_nodes_index = angle_value - offset + j;
							if (angle_compensate_nodes_index >= meas_nodes_with_compensation.size()) angle_compensate_nodes_index = meas_nodes_with_compensation.size() - 1;
							meas_nodes_with_compensation[angle_compensate_nodes_index] = meas_nodes[i];
						}
					}
				publish_scan(scan_pub, meas_nodes_with_compensation.data(), meas_nodes_with_compensation.size(), start_scan_time, scan_duration, inverted, 0.f, 359.f, max_distance, frame_id);
			}
		}

		//8.StopAndClean
		if (IS_FAIL(ret = drv->stop())) RCLCPP_ERROR(this->get_logger(), "Failed: stopScan and ret=%x", ret);
		else RCLCPP_INFO(this->get_logger(), "Done: stopScan");
		if (IS_FAIL(ret = drv->stopMotor())) RCLCPP_ERROR(this->get_logger(), "Failed: stopMotor and ret=%x", ret);
		else RCLCPP_INFO(this->get_logger(),  "Done: stopMotor");
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

