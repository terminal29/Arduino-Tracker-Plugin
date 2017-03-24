/** @date 2016
@author
Sensics, Inc.
<http://sensics.com/osvr>
*/

// Copyright 2016 Sensics Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>

// Generated JSON header file
#include "inf_arduino_tracker_json.h"

// Standard includes
#include <chrono>
#include <iostream>
#include <thread>
#include <string>
#include <windows.h>

// Library/third-party includes
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <serial/serial.h>
//#include "freepie_io.h"

#define PLUGIN_NAME "Arduino Tracker"

// Anonymous namespace to avoid symbol collision
namespace {

	class TrackerDevice {
	public:
		TrackerDevice(OSVR_PluginRegContext ctx) {

			// Create the initialization options
			OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

			// Configure our tracker
			osvrDeviceTrackerConfigure(opts, &m_tracker);

			// Create the device token with the options
			m_dev.initAsync(ctx, PLUGIN_NAME, opts);

			// Send JSON descriptor
			m_dev.sendJsonDescriptor(inf_arduino_tracker_json);

			// Register update callback
			m_dev.registerUpdateCallback(this);

			/*
			pieSlots = freepie_io_6dof_slots();
			*/

			std::string comPort;
			bool serialOpen = false;
			bool done = false;
			do {
				// Begin
				std::cout << "[]----------------------------- " << PLUGIN_NAME << " -----------------------------[]" << std::endl;
				std::cout << "[" << PLUGIN_NAME << "]: What COM port is your Arduino connected on? (ie. COM4)" << std::endl;
				
				// Get user port
				std::cin >> comPort;
				std::cout << "[" << PLUGIN_NAME << "]: Attempting to open port " << comPort << std::endl;
				
				//Create port object
				int attempts = 0;
				try {
					// Try open the serial
					d_serial = new serial::Serial(comPort, 115200, serial::Timeout::simpleTimeout(1000));

					if (d_serial->isOpen()) {
						serialOpen = true;
					}
				}
				catch (serial::PortNotOpenedException pnoe) {
					std::cout << "[" << PLUGIN_NAME << "]: Unable to open COM port." << std::endl;
				}
				catch (serial::IOException ioe) {
					std::cout << "[" << PLUGIN_NAME << "]: IO Error occurred." << std::endl;
				}
				
				//If serial has been opened but handshake has not been established...
				while (serialOpen && attempts < 3) {
					//try to send handshake
					std::cout << "[" << PLUGIN_NAME << "] Attempting handshake " << (attempts+1) << "/3" << std::endl;
					d_serial->write("Arduino Tracker Plugin Handshake\n");
					std::string result = d_serial->readline();
					if (result.compare("Hello from Arduino") == 0) {
						done = true;
						std::cout << "[" << PLUGIN_NAME << "]: Connection successful, resuming server loop." << std::endl;
						break; //break out of handshake loop.
					}
					attempts++;
				}

				//If the handshake has not completed properly by now, attempts should be at 3
				if (attempts >= 3) {
					std::cout << "[" << PLUGIN_NAME << "]: Error completing handshakes." << std::endl;
					std::cout << "[" << PLUGIN_NAME << "]: Check that the port is correct, and that the proper firmware has been loaded onto the microcontroller." << std::endl;
					std::cout << "[" << PLUGIN_NAME << "]: Try again? (Y/N)" << std::endl;
					std::string res;
					std::cin >> res;
					if (boost::iequals(res, "n")) {
						break;
					}
				
				}
				//If the serial is not open by now, something has gone wrong
				else if (!serialOpen) {
					std::cout << "[" << PLUGIN_NAME << "]: Error opening serial port." << std::endl;
					std::cout << "[" << PLUGIN_NAME << "]: Check that the port is correct, and that no other program is accessing that port." << std::endl;
					std::cout << "[" << PLUGIN_NAME << "]: Try again? (Y/N)" << std::endl;
					std::string res;
					std::cin >> res;
					if (boost::iequals(res, "n")) {
						break;
					}
				}


			} while (!done);
			enableSerial = true;
			osvrQuatSetIdentity(&m_state);
		}

		OSVR_ReturnCode update() {

			if (enableSerial) {

				std::vector<float> gyroData = getGyroOrientation();

				// If the arduino is sending rotation data
				if (boost::iequals(orientationFormat, "q")) {

					//Create Quaternion from Raw Data
					OSVR_Quaternion rotation = { gyroData.at(0), gyroData.at(1), gyroData.at(2), gyroData.at(3) };

					//Convert Quaternion to Axis-Angle format so we can rearrange the axes
					AxisAngle axisAngle = quat2AAngle(rotation);
					
					//Rearrange and load into our rotation state objects
					m_state = aAngle2Quat(axisAngle.a, -axisAngle.x, axisAngle.z, axisAngle.y);
				}


				/*
				uint32_t result = freepie_io_6dof_read(pieSlots, pieSlots, &pieData);
				if (result) {
					m_pos_state = {pieData.x, pieData.y, pieData.z};
					osvrDeviceTrackerSendPosition(m_dev, m_tracker, &m_pos_state, 0);
				}
				*/

				//Send data to osvr
				osvrDeviceTrackerSendOrientation(m_dev, m_tracker, &m_state, 0);
				loopCount++;
			}

			return OSVR_RETURN_SUCCESS;
		}

	private:
		osvr::pluginkit::DeviceToken m_dev;
		OSVR_TrackerDeviceInterface m_tracker;
		OSVR_Quaternion m_state;
		serial::Serial* d_serial;
		bool enableSerial = false;
		std::string orientationFormat = "q";
		struct AxisAngle { float a, x, y, z; };
		
		OSVR_PositionState m_pos_state;
		struct Vector3f { float x, y, z; };
		uint32_t pieSlots;
		/*freepie_io_6dof_data pieData;*/
		int loopCount = 0;

		std::vector<float> getGyroOrientation() {
			std::vector<float> gyroData = { 0,0,0,0 };
			std::string result;
			//Request gyroscope quaternion rotation
			try {
				d_serial->write(orientationFormat + "\n");

				//receive back gyroscope rotation in raw form
				result = d_serial->readline();
			}
			catch (serial::IOException e) {
				std::cout << "[" << PLUGIN_NAME << "]: Serial connection interrupted!" << std::endl;
				std::cout << "[" << PLUGIN_NAME << "]: Please restart OSVR server and reconnect to the tracker." << std::endl;
				// Kill the serial
				enableSerial = false;
				return gyroData;
			}
			/*
			if (result[0] == 'c') {
				std::cout << "[" << PLUGIN_NAME << "]: Tracker is calibrating, expect erratic movements until calibration is complete." << std::endl;
				do {
					result = d_serial->readline(); // sends a 'd' when its finished
				} while (!(result[0] == 'd' || result[0] == 'q'));
				std::cout << "[" << PLUGIN_NAME << "]: Tracker has finished calibrating, you are free to put on your headset again." << std::endl;
			
			}else */
			if (result[0] == 'q') {
				std::stringstream data(result);

				std::string separated;
				int iter = 0;
				std::getline(data, separated, ':'); //remove the q: from the start

				while (std::getline(data, separated, ':') && iter < 4) //4 quaternion rotations.
				{
					gyroData.at(iter) = atof(separated.c_str());
					iter++;
				}
			}

			return gyroData;
		}

		OSVR_Quaternion quatNormalize(OSVR_Quaternion &original) {
			float a, b, c, d;
			a = osvrQuatGetW(&original);
			b = osvrQuatGetX(&original);
			c = osvrQuatGetY(&original);
			d = osvrQuatGetZ(&original);
			double factor = std::sin(a / 2.0);
			// Calculate the x, y and z of the quaternion
			double x = b * factor;
			double y = c * factor;
			double z = d * factor;
			// Calculate the w value by cos( theta / 2 )
			double w = cos(a / 2.0);
			float len = std::sqrt(w*w + x*x + y*y + z*z);
			OSVR_Quaternion normalized = {w, x, y, z};
			return normalized;
		}

		AxisAngle quat2AAngle(OSVR_Quaternion &original) {
			OSVR_Quaternion inner = original;
			if (osvrQuatGetW(&inner) > 1) { // if w>1 acos and sqrt will produce errors, this cant happen if quaternion is normalised
				inner = quatNormalize(inner);
			}
			float angle = 2 * std::acosf(osvrQuatGetW(&inner));
			float s =std::sqrt(1 - osvrQuatGetW(&inner)*osvrQuatGetW(&inner)); // assuming quaternion normalised then w is less than 1, so term always positive.
			float x, y, z;
			if (s < 0.001) { // test to avoid divide by zero, s is always positive due to sqrt
							 // if s close to zero then direction of axis not important
			    x = osvrQuatGetX(&inner); // if it is important that axis is normalised then replace with x=1; y=z=0;
				y = osvrQuatGetY(&inner);
				z = osvrQuatGetZ(&inner);
			}
			else {
				x = osvrQuatGetX(&inner) / s; // normalise axis
				y = osvrQuatGetY(&inner) / s;
				z = osvrQuatGetZ(&inner) / s;
			}
			return{ angle, x, y, z };
		}

		OSVR_Quaternion aAngle2Quat(float a1, float x1, float y1, float z1) {
			float w, x, y, z;
			x = x1 * std::sin(a1 / 2);
			y = y1 * std::sin(a1 / 2);
			z = z1 * std::sin(a1 / 2);
			w = std::cos(a1 / 2);
			OSVR_Quaternion newQuat;
			osvrQuatSetW(&newQuat, w);
			osvrQuatSetX(&newQuat, x);
			osvrQuatSetY(&newQuat, y);
			osvrQuatSetZ(&newQuat, z);

			return newQuat;
		}

	};

	class HardwareDetection {
	public:
		HardwareDetection() : m_found(false) {}
		OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {

			if (m_found) {
				return OSVR_RETURN_SUCCESS;
			}

			// we always detect device in sample plugin
			m_found = true;

			// Create our device object
			osvr::pluginkit::registerObjectForDeletion(ctx, new TrackerDevice(ctx));

			return OSVR_RETURN_SUCCESS;
		}

	private:
		bool m_found;
	};
} // namespace

OSVR_PLUGIN(vpf_arbitrary_tracker) {

	osvr::pluginkit::PluginContext context(ctx);

	// Register a detection callback function object.
	context.registerHardwareDetectCallback(new HardwareDetection());

	return OSVR_RETURN_SUCCESS;
}
