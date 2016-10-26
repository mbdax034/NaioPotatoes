#include "Core.hpp"
#include <sys/types.h>

int main(int argc, char** argv ) {
	// core initialisation
	Core* core = new Core();
    
	// start main core thread
	core->init( DEFAULT_HOST_ADDRESS, static_cast<uint16_t>( PORT_ROBOT_MOTOR ) ) ;
    
	// waits the thread exits
	core->joinMainThread() ;

	delete core ;

	return 0 ;
}
