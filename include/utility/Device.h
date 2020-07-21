
#ifndef CUBESAT_DEVICE
#define CUBESAT_DEVICE

#include "agent/agentclass.h"
#include "support/configCosmos.h"
#include "agent/agentclass.h"
#include "support/jsonlib.h"
#include "support/jsonclass.h"

#include "cubesat_defs.h"
#include "utility/Exceptions.h"
#include "utility/DeviceDetail.h"
#include "utility/StringTools.h"

#include <unordered_map>


#pragma GCC diagnostic push
// Silence annoying warnings about pointer magic
#pragma GCC diagnostic ignored "-Wpointer-arith"


namespace cubesat {
	
	/**
	 * @brief A utility class for defining devices and their properties
	 */
	class Device {
	protected:
		//! Stores data for a COSMOS device property
		struct COSMOSProperty {
			std::string value_string;
			std::string cosmos_name;
			std::string readable_name;
			bool post;
			
			//! A default constructor is required for std::unordered_map
			COSMOSProperty() {}
			COSMOSProperty (const std::string &value_string, const std::string &cosmos_name, const std::string &readable_name, bool post)
				: value_string(value_string), cosmos_name(cosmos_name), readable_name(readable_name), post(post) {}
		};
		
		//! Stores data for a custom property
		struct CustomProperty {
			void *value_pointer;
			PropertyID value_type;
			
			//! A default constructor is required for std::unordered_map
			CustomProperty() {}
			
			template <typename T>
			CustomProperty(T value) {
				// Allocate memory for the value
				value_pointer = malloc(sizeof(value));
				
				// Set the value
				*(T*)value_pointer = value;
				
				// Store the value type
				value_type = GetPropertyID<T>();
			}
			
			template <typename T>
			bool TypeMatches() const {
				// Get the property ID of the given type to check if it matches
				return value_type == GetPropertyID<T>();
			}
		};
		
		
		
	public:
		//! A default constructor is needed for std::unordered_map
		Device()
			: agent(nullptr), type(DeviceType::NONE),
			  pindex(-1), cindex(-1), dindex(-1) {}
		Device(Agent *agent, DeviceType type, const std::string &device_name,
			   int pindex, int cindex, int dindex)
			: agent(agent), type(type), device_name(device_name),
			  pindex(pindex), cindex(cindex), dindex(dindex) {}
		~Device() {
			// Delete the custom properties
			for (auto pair : custom_properties)
				delete pair.second;
			custom_properties.clear();
		}
		
		//===============================================================
		//======================== MISCELLANEOUS ========================
		//===============================================================
		
		/**
		 * @brief Returns the name of this device, as supplied to SimpleAgent::NewDevice()
		 * @return The device name
		 */
		inline const std::string& GetName() const {
			return device_name;
		}		
		
		/**
		 * @brief Sets the UTC property to the current Modified Julian Date
		 * @tparam _DeviceType the type of device for this property (e.g. TemperatureSensor)
		 */
		template <typename _DeviceType>
		void Timestamp() {
			// Set the UTC property using the given device type
			SetProperty<typename _DeviceType::UTC>(currentmjd());
		}
		
		/**
		 * @brief (Internal) Adds all posted COSMOS properties the given list
		 * @param keys The list of properties to add to
		 */
		void StorePostedProperties(std::vector<std::string> &keys);
		
		//===============================================================
		//========================= PROPERTIES ==========================
		//===============================================================
		
		/**
		 * @brief Checks if a COSMOS property with the given name exists
		 * @tparam _DeviceProperty The device property (e.g. TemperatureSensor::Temperature)
		 * @return True if the property exists
		 */
		template <typename _DeviceProperty, typename T = typename _DeviceProperty::ValueType, size_t offset = _DeviceProperty::offset>
		typename std::enable_if<std::is_base_of<DeviceProperty<T, offset>, _DeviceProperty>::value, bool>::type
		PropertyExists() {
			// Check if the COSMOS property table has a key matching the property ID of the given property
			return cosmos_properties.find(GetPropertyID<_DeviceProperty>()) != cosmos_properties.end();
		}
		
		/**
		 * @brief Checks if a custom property with the given name exists
		 * @tparam T The storage type of the property (not necessary here)
		 * @param property_name The device property name
		 * @return True if the property exists
		 */
		template <typename T>
		bool PropertyExists(const std::string &property_name) {
#if SIMPLEAGENT_IGNORE_CASE
			for (auto property_pair : custom_properties) {
				if ( CompareAndIgnoreCase(property_name, property_pair.first) )
					return true;
			}
			return false;
#else
			
			// Check if the custom property table has a correspond key
			// Is the template parameter really necessary for this function?
			return custom_properties.find(property_name) != custom_properties.end();
#endif
		}
		
		/**
		 * @brief Adds and posts a property. Equivalent to SetProperty<...>(..., true)
		 * @tparam _DeviceProperty The property type
		 * @param value The value
		 */
		template <typename _DeviceProperty>
		void AddProperty(typename _DeviceProperty::ValueType value = typename _DeviceProperty::ValueType()) {
			// Set the property, and flag it as posted
			SetProperty<_DeviceProperty>(value, true);
		}
		
		/**
		 * @brief Sets the value of a COSMOS property
		 * @tparam _DeviceProperty The device property to set (e.g. TemperatureSensor::Temperature)
		 * @param value The value of the property. The type supplied should be trivially convertible to the once specified by _DeviceProperty
		 * @param post If true, this property is marked for posting. If this is false on subsequent calls, the property will remain posted
		 */
		template <typename _DeviceProperty>
		void SetProperty(typename _DeviceProperty::ValueType value, bool post = false) {
			
			// Check if there's an old value set, and if so, streal its 'post' value
			if ( cosmos_properties.find(GetPropertyID<_DeviceProperty>()) != cosmos_properties.end() ) {
				post = cosmos_properties[GetPropertyID<_DeviceProperty>()].post;
			}
			
			// Create the property
			COSMOSProperty new_property(ToString<typename _DeviceProperty::ValueType>(value), GetCOSMOSPropertyName<_DeviceProperty>(), _DeviceProperty::name, post);
			
			// Store the device property using the property ID
			cosmos_properties[GetPropertyID<_DeviceProperty>()] = new_property;
			
			// Set the property value using the DeviceProperty offset
			// Beware: pointer magic ahead!
			*(typename _DeviceProperty::ValueType*)((void*)&agent->cinfo->device[cindex] + _DeviceProperty::offset) = value;
		}
		
		/**
		 * @brief Sets a custom property for this device
		 * @tparam T The type of value to store
		 * @param name The name of the property
		 * @param value The value to store. If a pointer is stored, deleting the pointer elsewhere could cause problems
		 */
		template <typename T>
		void SetProperty(const std::string &name, T value) {
#if SIMPLEAGENT_IGNORE_CASE
			// Need to use the iterator
			for (auto it = custom_properties.begin(); it != custom_properties.end(); ++it) {
				if ( CompareAndIgnoreCase(name, it->first) ) {
					// Delete the old value
					delete it->second;
					// Remove the value from the table, since the case could be different
					custom_properties.erase(it);
					
					break;
				}
			}
			
			// Store the custom property
			custom_properties[name] = new CustomProperty(value);
#else
			// Check if the property is already set
			if ( custom_properties.find(name) != custom_properties.end() ) {
				// Delete the old value
				delete custom_properties[name];
			}
			
			// Store the custom property
			custom_properties[name] = new CustomProperty(value);
#endif
		}
		
		/**
		 * @brief Retrieves the value of a COSMOS property
		 * @tparam _DeviceProperty The device property to get (e.g. TemperatureSensor::Temperature)
		 * @return The value of the property, or a default-constructed value if the property doesn't exist
		 */
		template <typename _DeviceProperty>
		typename _DeviceProperty::ValueType GetProperty() {
			
			// Check if the property exists
			if ( !PropertyExists<_DeviceProperty>() ) {
				printf("Attempted to retrieve property '%s', which does not exist\n", _DeviceProperty::name);
				
				// Return a default-constructed value
				return typename _DeviceProperty::ValueType();
				
				//throw NonExistentPropertyException(_DeviceProperty::name);
			}
			else {
				// Get the property value using the DeviceProperty offset
				// Beware: pointer magic ahead!
				return *(typename _DeviceProperty::ValueType*)((void*)&agent->cinfo->device[cindex] + _DeviceProperty::offset);
			}
		}
		
		/**
		 * @brief Retrieves the value of a custom property
		 * @tparam T The storage type of the property. This NEEDS to match the one in SetProperty() to avoid type issues
		 * @param name The name of the property
		 * @return The property value
		 */
		template <typename T>
		T GetProperty(const std::string &name) {
#if SIMPLEAGENT_IGNORE_CASE
			for (auto property_pair : custom_properties) {
				if ( CompareAndIgnoreCase(name, property_pair.first) ) {
					
					// Make sure the given type is correct
					if ( !property_pair.second->TypeMatches<T>() )
						throw TypeMismatchException("The type supplied does not match the type stored");
					
					// Return the value supplied
					return *(T*)property_pair.second->value_pointer;
					
				}
			}
			
			// Return a default-constructed value
			return T();
#else
			if ( !PropertyExists<T>(name) ) {
				printf("Attempted to retrieve property '%s', which does not exist\n", name.c_str());
				//throw NonExistentPropertyException(name);
				return T();
			}
			else {
				
				// Look up the custom property
				CustomProperty *property = custom_properties[name];
				
				// Make sure the given type is correct
				if ( !property->TypeMatches<T>() )
					throw TypeMismatchException("The type supplied does not match the type stored");
				
				// Return the value supplied
				return *(T*)property->value_pointer;
			}
#endif
		}
		
		
		std::string GetPropertyStringByName(const std::string &cosmos_name) {
			std::string name_without_dindex;
			
			// Search the COSMOS properties
			for (auto device_pair : cosmos_properties) {
				// Get the name without the device index
				name_without_dindex = device_pair.second.cosmos_name.substr(0, device_pair.second.cosmos_name.length() - 4);
				if ( name_without_dindex == cosmos_name ) {
					return device_pair.second.value_string;
				}
			}
			
			return "";
		}
		
		//===============================================================
		//============================ DEBUG ============================
		//===============================================================
		
		/**
		 * @brief (Internal) Prints a list of stored properties for this device
		 * @param print_all If true, even non-posted and custom properties are listed
		 */
		void DebugPrint(bool print_all = false) const;
		
		/**
		 * @brief (Internal) Adds a list of stored properties for this device to a given stringstream
		 * @param ss The string stream to write to
		 * @param print_all If true, even non-posted and custom properties are listed
		 */
		void GetDebugString(std::stringstream &ss, bool print_all = false) const;
		
		
	protected:
		//! The agent this device belongs to
		Agent *agent;
		//! The COSMOS device type
		DeviceType type;
		//! The name of this device
		std::string device_name;
		//! COSMOS piece index
		int pindex;
		//! COSMOS component index
		int cindex;
		//! COSMOS device index
		int dindex;
		//! A table of custom properties added to this device
		std::unordered_map<std::string, CustomProperty*> custom_properties;
		//! A table of COSMOS properties added to this device
		std::unordered_map<PropertyID, COSMOSProperty> cosmos_properties;
		
		
		/**
		 * @brief Gets the COSMOS property name of a COSMOS property (e.g. device_tsen_temp_000)
		 * @tparam The device property (e.g. TemperatureSensor::Temperature)
		 * @return The COSMOS property name
		 */
		template <typename DeviceProperty>
		std::string GetCOSMOSPropertyName() {
			std::stringstream ss;
			
			// Format the COSMOS name as "device_deviceName_propertyName_deviceIndex"
			//                            [^^^^^^^^^^^^^^^] DeviceProperty::key
			ss << DeviceProperty::key << "_" << std::setw(3) << std::setfill('0') << dindex;
			return ss.str();
		}
	};
	
}

#pragma GCC diagnostic pop

#endif
