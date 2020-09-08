
#include "utility/SimpleAgent.h"

using namespace std;
using namespace cubesat;

//! An agent request which returns whatever argument the user gives it
double Request_Double(double arg);

//! An agent request which returns whatever we put in. The "CapturedInput"
//! type allows the user to put in any string they wish (including spaces)
std::string Request_Repeat(CapturedInput input);

//! A device request attached to a temperature sensor. Returns
//! the temperature plus a given number
float Request_GetTemperature(TemperatureSensor *sensor, int x);



//! Our SimpleAgent object
SimpleAgent *agent;
//! Our temperature sensor device
TemperatureSensor *temp_sensor;


int main() {
	
	// Create the agent
	agent = new SimpleAgent("my_agent");
	
	// Add the Request_Repeat request using the name "repeat"
	agent->AddRequest("repeat", Request_Repeat);
	
	// Add the "Request_Double" request using aliases "double" and "twice"
	agent->AddRequest({"double", "twice"}, Request_Double, "Doubles a number");
	
	
	
	// Add a temperature sensor device
	temp_sensor = agent->NewDevice<TemperatureSensor>("temp_sensor");
	
	// Set the utc and temperature properties to zero and post them
	temp_sensor->Post(temp_sensor->utc = 0);
	temp_sensor->Post(temp_sensor->temperature = 0);
	
	// Add the Request_GetTemperature request to the temperature sensor with
	// aliases "gettemp" and "temp"
	temp_sensor->AddRequest({"gettemp", "temp"}, Request_GetTemperature);
	
	
	
	// Let the agent know all the devices have been set up
	agent->Finalize();
	
	// Show all of the properties and requests added
	agent->DebugPrint();
	
	// A counter which holds a dummy value for the temperature
	int i = 0;
	
	// Start running the agent
	while ( agent->StartLoop() ) {
		
		// Timestamp the device
		temp_sensor->utc = currentmjd();
		temp_sensor->temperature = i++;
	}
	
	return 0;
}



double Request_Double(double x) {
	return x * 2;
}
float Request_GetTemperature(TemperatureSensor *sensor, int x) {
	return sensor->temperature + x;
}
std::string Request_Repeat(CapturedInput input) {
	return input.input;
}
