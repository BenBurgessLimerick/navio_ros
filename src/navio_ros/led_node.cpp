#include <Navio2/Led_Navio2.h>
#include <Common/Util.h>

#include <iostream>

int main(int argc, const char* argv[]) {
	Led_Navio2* led = new Led_Navio2();

	if (!led->initialize()) {
		std::cout << "Failure" << std::endl;
	}

	while(true) {
		led->setColor(Colors::Green);
		std::cout << "Led is green" << std::endl;
		sleep(1);

		led->setColor(Colors::Red);
		std::cout << "Led is red" << std::endl;
		sleep(1);
	}
	return 0;
}
