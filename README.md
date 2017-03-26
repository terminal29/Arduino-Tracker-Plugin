# Arduino-Tracker-Plugin
Plugin for OSVR that uses an Arduino + MPU6050 to do rotational headtracking. Very unstable so use at your own risk :p

**This guide and plugin is a work in progress so dont expect everything to be buttery smooth just yet.**

* Arduino Code is just a slight rewrite of https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
	and requires i2c and mpu6050 libs from the above repo to be installed.

* Using Serial Library from https://github.com/wjwwood/serial

* (Soon to be) Using FreePIE IO Library from https://github.com/AndersMalmgren/FreePIE/tree/master/Lib/IO

# How to use
1. Load up the arduino IDE and make sure you have installed the required libs (https://www.arduino.cc/en/Guide/Libraries) then open up the *Arduino_Tracker_Sketch.ino* file and upload it to your board. Once it is done, open up the serial input and set line endings to *Newline* and serial speed to *115200 baud*. Now when you type anything into the input,
If you get back:

		Hello from Arduino

	then you are all set.
	
2. Copy the *inf_arduino_tracker.dll* and *inf_arduino_tracker.json* files into the *osvr-plugins-0* folder of your OSVR install directory (You do not need to modify these files).



3. In the main OSVR binary folder (where your osvr_server.exe is), there should be a file called *osvr_server_config.json*. You want to open this file with notepad++ or your choice of text editor, and add or replace the line in "aliases" with:
	
	```		
	"aliases": {
		"/me/head": "/inf_arduino_tracker/Arduino Tracker/semantic/hmd"
	}
	```
	for example, my server config looked like this:
	```
	{
		"display": "displays/Oculus_Rift_DK1.json",
		"renderManagerConfig": "renderManager.extended.landscape.json"
	}
	```
	and now looks like this:
	```
	{
		"display": "displays/Oculus_Rift_DK1.json",
		"renderManagerConfig": "renderManager.extended.landscape.json",
		"aliases": {
		"/me/head": "/inf_arduino_tracker/Arduino Tracker/semantic/hmd"
		}
	}
	```

4. Go to device manager and find out which COM port your arduino is connected on

5. Run OSVR and type in the com port (ie com4 you would type *COM4* etc...)

6. Wait for it to connect and test tracking with one of the osvr tracking test programs or the tracker viewer. 

**notes**

The X axis of your MPU should be pointing toward the left, and y axis pointing toward you and be mounted on the front of your headset for the axes to align.

If your axes are slightly off, run the calibration script in the MPU6050 library examples and find your offsets, then modify the existing offsets in the Arduino Tracker Sketch file and reupload to your board.
