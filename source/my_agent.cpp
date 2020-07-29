
#include "utility/SimpleAgent.h"

using namespace std;
using namespace cubesat;

//! A request which returns a friendly greeting
string Request_SayHi();
//! A request which returns whatever argument the user gives it
string Request_Echo(vector<string> args);


//! Our SimpleAgent object
SimpleAgent *agent;
//! Our temperature sensor device
TemperatureSensor *temp_sensor;

int main() {
	
	// Create the agent
	agent = new SimpleAgent("my_agent");
	
	
	// As an example, we can add a temperature sensor device
	temp_sensor = agent->NewDevice<TemperatureSensor>("temp_sensor");
	
	auto imu = agent->NewDevice<IMU>("imu");
	imu->Post(imu->magnetic_field);
	
	// Add a couple of COSMOS properties
	temp_sensor->Post(temp_sensor->utc);
	temp_sensor->Post(temp_sensor->temperature);
	temp_sensor->utc = 0;
	temp_sensor->temperature = 0;
	
	
	// Add the "say_hi" request with aliases "say_hi" and "greeting"
	agent->AddRequest({"say_hi", "greeting"}, Request_SayHi, "Says hi");
	
	// Add the "echo" request
	agent->AddRequest("repeat", Request_Echo, "Repeats what you put in", "I don't know what space is *hint hint*");
	
	// Let the agent know all the devices have been set up
	agent->Finalize();
	
	// Show all of the properties and requests added
	agent->DebugPrint(true);
	
	int i = 0;
	
	// Start running the agent
	while ( agent->StartLoop() ) {
		
		imu->magnetic_field = Vec3(0, i, 0);
		
		// Timestamp the device
		temp_sensor->utc = currentmjd();
		temp_sensor->temperature = i++;
	}
	
	return 0;
}

string Request_SayHi() {
	return "Hi there!";
}
string Request_Echo(vector<string> args) {
	if ( args.size() != 1 )
		return "Usage: echo word";
	else
		return args[0];
}
