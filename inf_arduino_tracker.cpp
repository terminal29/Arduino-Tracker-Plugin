// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>
#include <osvr/PluginKit/ButtonInterfaceC.h>

// Standard includes
#include <thread>
#include <iostream>
#include <atomic>
#include <mutex>

// Generated JSON header file
#include "inf_arduino_tracker_json.h"

// Third party includes
#include <json/json.h>

// etc.
#include "common.h"

// Anonymous namespace to avoid symbol collision
namespace {

	std::atomic<float> q_w = 0, q_x = 0, q_y = 0, q_z = 0;
	std::atomic<bool> keep_connection = true;
	std::mutex msg_queue_lock;
	std::vector<std::string> msg_queue;

	void serial_connect(std::string port) {
		// serial connect
		while (keep_connection) {
			std::string recv_buffer;
			//receive line(&recv_buffer);
			// Received Quaternion data
			switch(recv_buffer.c_str()[0]) {
			case 'Q': //Received quaternion data
				float p_w, p_x, p_y, p_z;
				// parse string


				break;
			case 'C': //Calibration start or end
				msg_queue_lock.lock();
				msg_queue.push_back(recv_buffer);
				msg_queue_lock.unlock();

				break;
			case 'D': //Received descriptor (Information)
				msg_queue_lock.lock();
				msg_queue.push_back(recv_buffer);
				msg_queue_lock.unlock();

				break;
			default:

			}
		}
	}

	class Arduino_Tracker_Device {
	public:
		Arduino_Tracker_Device(OSVR_PluginRegContext ctx) {

			// Create the initialization options
			OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);
			

			
		}

		OSVR_ReturnCode update() {
			if (msg_queue.size() > 0) {
				msg_queue_lock.lock();
				int i = msg_queue.size() - 1;
				while (msg_queue.size() > 0) {
					std::cout << msg_queue.at(i) << std::endl;
					msg_queue.pop_back();
					i--;
				}

				//set quaternion
			}

		return OSVR_RETURN_SUCCESS;
		}
		~Arduino_Tracker_Device() {
			keep_connection = false;
		}
	private:
		
	};

	class Arduino_Tracker_Constructor {
	public:
		OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx, const char *params) {
			std::cout << PRNT_PFX << " Loading config..." << std::endl;
			std::string com_port;
			Json::Value config_params;
			if (params) {
				Json::Reader reader;
				bool parse_result = reader.parse(params, config_params);
				if (!parse_result) {
					std::cout << PRNT_PFX << " Invalid config, load failed!" << std::endl;
					return OSVR_RETURN_FAILURE;
				}
				com_port = config_params.get("port", "COM0").asString();
			}
			else {
				// No config params so we assume the user has our plugin installed just isn't using it
				return OSVR_RETURN_SUCCESS;
			}
			std::thread serial_thread = std::thread(serial_connect, com_port);


			
			osvr::pluginkit::registerObjectForDeletion(ctx, new Arduino_Tracker_Device(ctx));
			return OSVR_RETURN_SUCCESS;
		}
	};
		
} // namespace

OSVR_PLUGIN(inf_arduino_tracker) {
	osvr::pluginkit::PluginContext context(ctx);
	osvr::pluginkit::registerDriverInstantiationCallback(ctx, DEVICE_NAME, new Arduino_Tracker_Constructor);
	return OSVR_RETURN_SUCCESS;
}
