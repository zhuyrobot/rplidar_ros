#include <signal.h>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "rplidar.h"

using namespace std;
using namespace rp::standalone::rplidar;

class RPLidarScanPublisher : public rclcpp::Node
{
public:
	u_result ret = 0;
	rp::standalone::rplidar::RPlidarDriver* drv = NULL;
	RPLidarScanPublisher() : Node("rplidar_scan_publisher") {}

public:
	string channel_type = this->declare_parameter("channel_type", "serial");//serial/tcp
	string serial_port = this->declare_parameter("serial_port", "/dev/ttyUSB0");
	int serial_baudrate = this->declare_parameter("serial_baudrate", 115200/*256000*/);//ros run for A1 A2, change to 256000 if A3
	string tcp_ip = this->declare_parameter("tcp_ip", "192.168.0.7");
	int tcp_port = this->declare_parameter("tcp_port", 20108);
	string frame_id = this->declare_parameter("frame_id", "laser");
	string scan_mode = this->declare_parameter("scan_mode", "");//Standard/Express/Boost/Sensitivity/Stability
	bool inverted = this->declare_parameter("inverted", false);
	bool angle_compensate = this->declare_parameter("angle_compensate", true);//使扫描点均匀地分布在360度范围, 步长通常设为1度, RPLidar角分辨最小0.45度最大1.35度典型0.9度, 所以可设置每个步长内分布1或2个扫描点
	int log_detail = this->declare_parameter("log_detail", 0); //1:RawScan   2:FormatScan   3:BothScan

public:
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

private:
	static float get_angle_deg(const rplidar_response_measurement_node_hq_t& node) { return node.angle_z_q14 * 90.f / 16384.f; }
	void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub,
		rplidar_response_measurement_node_hq_t* meas_nodes, int node_count,
		rclcpp::Time scan_timestamp, string frame_id, double scan_duration, bool inverted,
		float angle_min, float angle_max, float range_max, float range_min = 0.15)
	{
		//0.
		static int scan_count = 0; ++scan_count;
		auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

		//1.
		scan_msg->header.stamp = scan_timestamp;
		scan_msg->header.frame_id = frame_id;
		scan_msg->scan_time = scan_duration;
		scan_msg->time_increment = scan_duration / (node_count - 1);

		//2.
		bool reversed = (angle_max > angle_min);
		if (!reversed) RCLCPP_INFO(this->get_logger(), "Warn: line%d should never happen", __LINE__);
		scan_msg->angle_min = M_PI - (reversed ? angle_max : angle_min);
		scan_msg->angle_max = M_PI - (reversed ? angle_min : angle_max);
		scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (node_count - 1);

		//3.
		scan_msg->range_min = range_min;
		scan_msg->range_max = range_max;
		scan_msg->ranges.resize(node_count);
		scan_msg->intensities.resize(node_count);
		bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
		for (int i = 0; i < node_count; ++i)
		{
			float distance = float(meas_nodes[i].dist_mm_q2 * 0.00025);//=1./4/1000;
			int index = reverse_data ? node_count - 1 - i : i;
			scan_msg->ranges[index] = distance != 0 ? distance : numeric_limits<float>::infinity();
			scan_msg->intensities[index] = float(meas_nodes[i].quality >> 2);
		}
		pub->publish(*scan_msg);
	}

public:
	int work_loop(bool &exit)
	{
		//1.CreateDriver
		RCLCPP_INFO(this->get_logger(), "RPLidar_SDK_Version: %s\nRPLidar_ROS2_Version: 1.0.1 ", RPLIDAR_SDK_VERSION);
		if (channel_type == "tcp") drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_TCP);
		else drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
		if (!drv) { RCLCPP_ERROR(this->get_logger(), "Failed: create driver"); return -2; }
		else RCLCPP_INFO(this->get_logger(), "Done: create driver");

		//2.ConnectLidar
		if (channel_type == "tcp") ret = drv->connect(tcp_ip.c_str(), uint32_t(tcp_port));
		else ret = drv->connect(serial_port.c_str(), uint32_t(serial_baudrate));
		string connstr = fmt::format("connect {} with {}", channel_type == "tcp" ? tcp_ip : serial_port, channel_type == "tcp" ? tcp_port : serial_baudrate);
		if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: %s and ret=%x", connstr.c_str(), ret); RPlidarDriver::DisposeDriver(drv); return -1; }
		else RCLCPP_INFO(this->get_logger(), "Done: %s", connstr.c_str());

		//3.DeviceInfo
		rplidar_response_device_info_t device_info;
		if (IS_FAIL(ret = drv->getDeviceInfo(device_info))) { RCLCPP_ERROR(this->get_logger(), "Failed: getDeviceInfo and ret=%x", ret); RPlidarDriver::DisposeDriver(drv); return -1; }
		else
		{
			string strsn; for (int k = 0; k < 16; ++k) strsn += fmt::format("{:02X}", device_info.serialnum[k]);
			RCLCPP_INFO(this->get_logger(), "RPLidarSN: %s", strsn.c_str());
			RCLCPP_INFO(this->get_logger(), "FirmwareVer: %d.%02d", device_info.firmware_version >> 8, device_info.firmware_version & 0xFF);
			RCLCPP_INFO(this->get_logger(), "HardwareRev: %d", int(device_info.hardware_version));
		}

		//4.DeviceHealth
		rplidar_response_device_health_t device_health;
		if (IS_FAIL(ret = drv->getHealth(device_health))) { RCLCPP_ERROR(this->get_logger(), "Failed: getHealth and ret=%x", ret); RPlidarDriver::DisposeDriver(drv); return -1; }
		else RCLCPP_INFO(this->get_logger(), "HealthStatus : %d", device_health.status);

		//5.ScanModes
		vector<RplidarScanMode> all_scan_modes;
		if (IS_FAIL(drv->getAllSupportedScanModes(all_scan_modes))) { RCLCPP_ERROR(this->get_logger(), "Failed: getAllSupportedScanModes and ret=%x", ret); RPlidarDriver::DisposeDriver(drv); return -1; }
		else for (int k = 0; k < all_scan_modes.size(); ++k)
			RCLCPP_INFO(this->get_logger(), "SupportScanMode%d: name=%s, MaxDistance=%.1fm, SamplePerUS=%.1f, PointPerDegreee=%d",//can scan how many points per degree//10HZ//360deg
				all_scan_modes[k].id, all_scan_modes[k].scan_mode, all_scan_modes[k].max_distance, 1 / all_scan_modes[k].us_per_sample, int(1000 * 1000 / all_scan_modes[k].us_per_sample / 10.0 / 360.0));

		//6.StartMotor
		if (IS_FAIL(ret = drv->startMotor())) { RCLCPP_ERROR(this->get_logger(), "Failed: startMotor"); RPlidarDriver::DisposeDriver(drv); return -1; }
		else RCLCPP_INFO(this->get_logger(), "Done: startMotor");

		//7.StartScan
		size_t npoint_per_degree;
		RplidarScanMode acutal_scan_mode;
		if (scan_mode.empty()) ret = drv->startScan(false, true, 0, &acutal_scan_mode);
		else//Custom mode
		{
			uint16_t scan_id = -1;
			for (int k = 0; k < all_scan_modes.size(); ++k) if (all_scan_modes[k].scan_mode == scan_mode) { scan_id = all_scan_modes[k].id; break; }
			if (scan_id != -1) ret = drv->startScanExpress(false, scan_id, 0, &acutal_scan_mode);
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Failed: scan mode %s is not supported", scan_mode.c_str());
				drv->stopMotor(); RPlidarDriver::DisposeDriver(drv); return -1;
			}
		}
		if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: startScan/startScanExpress and ret=%x", ret); drv->stopMotor(); RPlidarDriver::DisposeDriver(drv); return -1; }
		else
		{
			npoint_per_degree = int(1000 * 1000 / acutal_scan_mode.us_per_sample / 10.0 / 360.0);
			if (npoint_per_degree < 1) npoint_per_degree = 1;
			RCLCPP_INFO(this->get_logger(), "CurrentScanMode%d: name=%s, MaxDistance=%.1fm, SamplePerUS=%.1f, PointPerDegreee=%d", 
				acutal_scan_mode.id, acutal_scan_mode.scan_mode, acutal_scan_mode.max_distance, 1 / acutal_scan_mode.us_per_sample, npoint_per_degree);
		}

		//8.SpinROS
		for(int64_t frameId = 0; rclcpp::ok() && !exit; ++frameId)
		{
			rclcpp::spin_some(shared_from_this());

			//8.1 GrabScan
			size_t meas_count = 360 * 8;
			vector<rplidar_response_measurement_node_hq_t> meas_nodes(meas_count);
			rclcpp::Time scan_timestamp = this->now();
			ret = drv->grabScanDataHq(meas_nodes.data(), meas_count);
			double scan_duration = (this->now() - scan_timestamp).seconds();
			if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: grabScanDataHq and ret=%x", ret); continue; }

			//8.2 SortScan
			ret = drv->ascendScanData(meas_nodes.data(), meas_count);
			if (IS_FAIL(ret)) { RCLCPP_ERROR(this->get_logger(), "Failed: ascendScanData and ret=%x", ret); continue; }

			//8.3 RawScan
			if (angle_compensate == false || log_detail & 1)
			{
				int first_node = -1;
				int final_node = meas_count;
				while (meas_nodes[++first_node].dist_mm_q2 == 0);//First valid node
				while (meas_nodes[--final_node].dist_mm_q2 == 0);//Final valid node
				float angle_min = get_angle_deg(meas_nodes[first_node]) * M_PI / 180; //rad
				float angle_max = get_angle_deg(meas_nodes[final_node]) * M_PI / 180; //rad
				publish_scan(scan_pub, meas_nodes.data() + first_node, final_node - first_node + 1, scan_timestamp, frame_id, scan_duration, inverted, angle_min, angle_max, acutal_scan_mode.max_distance);
				string scan_detail = fmt::format("RawScan{}: AngleMin={}, AngleMax={}, PointCount={}", frameId, angle_min, angle_max, final_node - first_node + 1);
				if (log_detail & 1) for (int k = first_node; k < final_node; ++k) scan_detail += fmt::format("\n\tPoint{}: [ {}, {} ]", k, get_angle_deg(meas_nodes[k]), meas_nodes[k].dist_mm_q2 * 0.00025);
				if (long(time(0)) % 2 == 0) RCLCPP_INFO(this->get_logger(), "%s", scan_detail.c_str());
			}

			//8.4 FormatScan
			if (angle_compensate == true || log_detail & 2)
			{
				vector<rplidar_response_measurement_node_hq_t> formated_meas_nodes(360 * npoint_per_degree);
				memset(formated_meas_nodes.data(), 0, formated_meas_nodes.size() * sizeof(formated_meas_nodes[0]));
				//If meas_count<360*npoint_per_degree: formated_meas_nodes includes some continuously repeated measurements
				for (size_t i = 0, offset = 0; i < meas_count; ++i)
					if (meas_nodes[i].dist_mm_q2 != 0) //has been zero defaultly if zero distance
					{
						int angle2index = int(get_angle_deg(meas_nodes[i]) * npoint_per_degree);
						if (offset > angle2index) { offset = angle2index; RCLCPP_WARN(this->get_logger(), "Warn: line%d should not happen", __LINE__); }
						for (size_t k = 0; k < npoint_per_degree; ++k)
						{
							int index2offset = angle2index - offset + k;
							if (index2offset >= formated_meas_nodes.size()) { index2offset = formated_meas_nodes.size() - 1; RCLCPP_WARN(this->get_logger(), "Warn: line%d should not happen often", __LINE__); }
							formated_meas_nodes[index2offset] = meas_nodes[i];
						}
					}
				publish_scan(scan_pub, formated_meas_nodes.data(), formated_meas_nodes.size(), scan_timestamp, frame_id, scan_duration, inverted, 0.f, float(359.f * M_PI / 180), acutal_scan_mode.max_distance);
				string scan_detail = fmt::format("FormatScan{}: PointPerDegree={}, PointCount={}", frameId, npoint_per_degree, formated_meas_nodes.size());
				if (log_detail & 2) for (int k = 0; k < formated_meas_nodes.size(); ++k) scan_detail += fmt::format("\n\tPoint{}: [ {}, {} ]", k, get_angle_deg(formated_meas_nodes[k]), formated_meas_nodes[k].dist_mm_q2 * 0.00025);
				if (long(time(0)) % 2 == 0) RCLCPP_INFO(this->get_logger(), "%s", scan_detail.c_str());
			}
		}

		//9.StopAndClean
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
	static bool exit = false;
	signal(SIGINT, [](int sig)->void { exit = true; });
	int ret = rplidar_scan_publisher->work_loop(exit);
	rclcpp::shutdown();
	return ret;
}

