#include "Core.hpp"
#include "robot.hpp"

#include <fstream>

using namespace std;


int main(int argc, char** argv ) {
	// core initialisation
	Core* core = new Core();
	
	printNull();
	// start main core thread
	core->init( DEFAULT_HOST_ADDRESS, static_cast<uint16_t>( PORT_ROBOT_MOTOR ) ) ;

	 // waits the thread exits
	core->joinMainThread() ;

	ofstream file;
    file.open("test.text", ios::out | ios::app);
    file << "je teste";
    file.close();
	delete core ;

	return 0 ;
}
