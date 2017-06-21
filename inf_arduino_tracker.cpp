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

			
		}

		OSVR_ReturnCode update() {


		return OSVR_RETURN_SUCCESS;
		}

	private:
		osvr::pluginkit::DeviceToken m_dev;
		OSVR_TrackerDeviceInterface m_tracker;
		OSVR_Quaternion m_state;
		serial::Serial* d_serial;

		struct AxisAngle { float a, x, y, z; };
		
		OSVR_PositionState m_pos_state;
		struct Vector3f { float x, y, z; };


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
