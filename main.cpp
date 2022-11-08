#include "simulator.h"

int main()
{
	std::cout<<"---------- PID PATH TRACKING ------------"<<std::endl;

	pt::Simulator sim;
	
	sim.initTrajectory();
	sim.process();	

	return 0;
}
