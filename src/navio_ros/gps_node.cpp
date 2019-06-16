#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <Common/Ublox.h>
#include <Common/Util.h>

using namespace std;

int main(int argc, char* argv[]) {
	if (check_apm()) {
		ROS_ERROR("Check apm returned not 0\n");
		return 1;
	}

	ros::init(argc, argv, "navio_gps_node");
	ros::NodeHandle n;
	ros::Publisher gps_publisher = n.advertise<sensor_msgs::NavSatFix>("navio_gps", 10);

	std::vector<double> pos_data;

	Ublox gps;

	sensor_msgs::NavSatFix msg;
	sensor_msgs::NavSatStatus gps_status;

	if (gps.testConnection()) {
		std::cout << "Ublox test ok" << std::endl;

		if (!gps.configureSolutionRate(1000)) {
			std::cout << "Setting new rate: FAILED" << std::endl;
		}

		while (ros::ok()) {
			if (gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data) == 1) {

				msg.status = gps_status;


				msg.latitude = pos_data[2] / 10000000.0;
				msg.longitude = pos_data[1] / 10000000.0;
				msg.altitude = pos_data[3] / 1000.0;

				//Fill in the diagonal
				msg.position_covariance[0] = pos_data[5] / 1000.0;
				msg.position_covariance[4] = pos_data[5] / 1000.0;
				msg.position_covariance[8] = pos_data[6] / 1000.0;
				msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

				

				// std::cout << "Long " << pos_data[1] / 10000000 << std::endl;
				// std::cout << "Lat: " << pos_data[2] / 10000000 << std::endl;
			}

			if (gps.decodeSingleMessage(Ublox::NAV_STATUS, pos_data) == 1) {
				// std::cout << "GPS fix : " << pos_data[1] << std::endl;

				switch((int) pos_data[0]) {
					case 0x00:
						gps_status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
						gps_status.service = sensor_msgs::NavSatStatus::SERVICE_GPS; // not sure this is true
						break;
					default: // should check its one of 0x01, 0x02, 0x03, 0x04, 0x05
						gps_status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
						gps_status.service = sensor_msgs::NavSatStatus::SERVICE_GPS; // not sure this is true
						break;
				}	
			}
		}
	}
	return 0;
}
