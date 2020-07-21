
#include "utility/Device.h"

using namespace std;
using namespace cubesat;



void Device::StorePostedProperties(std::vector<std::string> &keys) {
	
	// Go through each COSMOS property
	for (auto property_pair : cosmos_properties) {
		
		// Check if the property should be posted
		if ( property_pair.second.post ) {
			
			// Add the COSMOS name for the property
			keys.push_back(property_pair.second.cosmos_name);
		}
	}
}

//===============================================================
//============================ DEBUG ============================
//===============================================================

void Device::DebugPrint(bool print_all) const {
	
	// Print the COSMOS properties
	for (auto property_pair : cosmos_properties) {
		if ( print_all || property_pair.second.post )
			printf("|\t|\t| COSMOS Property '%s' (aka %s): %s\n",
				   property_pair.second.readable_name.c_str(),
				   property_pair.second.cosmos_name.c_str(),
				   property_pair.second.value_string.c_str());
	}
	
	// Print the custom properties
	if ( print_all ) {
		for (auto property_pair : custom_properties) {
			printf("|\t|\t| Custom Property '%s'\n", property_pair.first.c_str());
		}
	}
}

void Device::GetDebugString(std::stringstream &ss, bool print_all) const {
	// Print the COSMOS properties
	for (auto property_pair : cosmos_properties) {
		if ( print_all || property_pair.second.post )
			ss << "|\t|\t| COSMOS Property '" << property_pair.second.readable_name <<
				  "' (aka " << property_pair.second.cosmos_name <<
				  "): " << property_pair.second.value_string << "\n";
	}
	
	// Print the custom properties
	if ( print_all ) {
		for (auto property_pair : custom_properties) {
			ss << "|\t|\t| Custom Property '" << property_pair.first << "'\n";
		}
	}
}

//===============================================================
//============================ OTHER ============================
//===============================================================
const char* cubesat::GetDeviceTypeString(DeviceType type) {
	switch ( type ) {
		case DeviceType::PLOAD: return "payload";
		case DeviceType::SSEN: return "sun sensor";
		case DeviceType::TSEN: return "temperature sensor";
		case DeviceType::MTR: return "magnetic torque rod";
		case DeviceType::ANT: return "antenna";
		case DeviceType::CPU: return "CPU";
		case DeviceType::IMU: return "IMU";
		case DeviceType::GPS: return "GPS";
		case DeviceType::RW: return "reaction wheel";
		case DeviceType::RXR: return "radio receiver";
		case DeviceType::TXR: return "radio transmitter";
		case DeviceType::TCV: return "radio transceiver";
		case DeviceType::MOTR: return "motor";
		case DeviceType::THST: return "thruster";
		case DeviceType::PROP: return "propellant tank";
		case DeviceType::SWCH: return "switch";
		case DeviceType::ROT: return "rotor";
		case DeviceType::STT: return "star tracker";
		case DeviceType::MCC: return "motion capture camera";
		case DeviceType::TCU: return "torque rod control unit";
		case DeviceType::BUS: return "power bus";
		case DeviceType::PSEN: return "pressure sensor";
		case DeviceType::CAM: return "camera";
		case DeviceType::TELEM: return "telemetry";
		case DeviceType::DISK: return "disk drive";
		default: return "invalid";
	}
}

