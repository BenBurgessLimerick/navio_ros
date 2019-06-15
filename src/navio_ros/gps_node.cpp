#include <Common/Ublox.h>
#include <Common/Util.h>

using namespace std;

int main(int argc, char* argv[]) {
	if (check_apm()) {
		return 1;
	}

	std::vector<double> pos_data;

	Ublox gps;

	if (gps.testConnection()) {
		std::cout << "Ublox test ok" << std::endl;

		if (!gps.configureSolutionRate(1000)) {
			std::cout << "Setting new rate: FAILED" << std::endl;
		}

		while (true) {
			if (gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data) == 1) {
				std::cout << "Long " << pos_data[1] / 10000000 << std::endl;
				std::cout << "Lat: " << pos_data[2] / 10000000 << std::endl;
			}

			if (gps.decodeSingleMessage(Ublox::NAV_STATUS, pos_data) == 1) {
				std::cout << "GPS fix : " << pos_data[1] << std::endl;
			}
		}
	}
	return 0;
}
