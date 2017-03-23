# Arduino-Tracker-Plugin
Plugin for OSVR that uses an Arduino + MPU to do rotational headtracking. Very unstable so use at your own risk :p

**This guide and plugin is a work in progress so dont expect everything to be buttery smooth just yet.**

* Arduino Code is just a slight rewrite of https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050 

* Using Serial Library from https://github.com/wjwwood/serial

* (Soon to be) Using FreePIE IO Library from https://github.com/AndersMalmgren/FreePIE/tree/master/Lib/IO

# How to use
1. Assuming you have connected up the MPU to your Arduino properly, upload the Arduino script to the arduino, and test it by opening the serial at 115200 bitrate, and setting line endings to "Newline", and typing anything into the serial input. 
If you get back:

		Hello from Arduino

	then you are all set.
	
2. Copy the *Arduino_Tracker_Plugin.dll* and *Arduino_Tracker_Plugin.json* files into the *osvr-plugins-0* folder of your OSVR install directory.



3. If you are using a generic HDMI output, edit your *osvr-server-config.json* to include:

		"display": "displays/Oculus_Rift_DK1.json",
		"renderManagerConfig": "sample-configs/renderManager.extended.landscape.json",
		"aliases": {
			"/me/head": "/vpf_arbitrary_tracker/Arbitrary Tracker Plugin/semantic/hmd"
		}

	or if you are using some other pre-configured hmd, just include:

		"aliases": {
			"/me/head": "/vpf_arbitrary_tracker/Arbitrary Tracker Plugin/semantic/hmd"
		}
	**These names will be changing in the next release so be sure to check back here when you update**
4. Go to device manager and find out which COM port your arduino is connected on

5. Run OSVR and type in the com port (ie com4 you would type *COM4* etc...)

6. Wait for it to connect and test tracking with one of the osvr tracking test programs or the tracker viewer
