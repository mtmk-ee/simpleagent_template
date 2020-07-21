
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
Device *temp_sensor;

int main() {
	
	// Create the agent
	agent = new SimpleAgent("my_agent");
	
	
	// As an example, we can add a temperature sensor device
	temp_sensor = agent->NewDevice<TemperatureSensor>("temp_sensor");
	
	// Add a couple of COSMOS properties
	temp_sensor->AddProperty<TemperatureSensor::UTC>();
	temp_sensor->AddProperty<TemperatureSensor::Temperature>();
	
	// Add a custom property
	temp_sensor->SetProperty<int>("counter", 0);
	
	
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
		
		// Timestamp the device
		temp_sensor->Timestamp<TemperatureSensor>();
		
		// Set the temperature property to some value
		temp_sensor->SetProperty<TemperatureSensor::Temperature>(273.15 + i * 2);
		
		// Set the counter property
		temp_sensor->SetProperty<int>("counter", i);
		
		++i;
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
