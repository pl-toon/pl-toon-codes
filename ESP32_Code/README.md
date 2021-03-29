## Index
- [Programming the Agents](#how-to-program-the-agents)
- [Running the Code](#running-the-code)
	* [Initial Setup](#initial-setup)
	* [Experiments](#performing-an-experiment)
---

## How to Program the Agents

The code is developed for the ESP32 board using the Arduino IDE libraries to compile the code. The most straightforward method to compile and run the codes, is to install the ESP32 libraries for Arduino and upload the code to the board within the same IDE. (You can watch how to do this [here](https://www.youtube.com/watch?v=mBaS3YnqDaU))

The [LeaderAgent](https://github.com/pl-toon/pl-toon-codes/tree/main/ESP32_Code/LeaderAgent) code has to be uploaded to the train that will go in front of the platoon, while the [PIDAgents](https://github.com/pl-toon/pl-toon-codes/tree/main/ESP32_Code/PIDAgents) code is meant to be used by the following agents that need to control their own speed and distance respect to the next agent in front.

In both cases you have to manually set in the code the `SSID` and `Password` of the Wi-Fi network to connect, as well as the `IP` and `Port` of the MQTT server.

***Note**: Before uploading the code, make sure to [calibrate the lens](https://github.com/pl-toon/pl-toon-codes/tree/main/ESP32_Code/Misc/Camera_Calibration) of the optical sensor in order to get a clear image on the checkerboard pattern and therefore obtain proper measurements of the velocity.*

---

## Running the Code
### Initial Setup

After powering on the agent. An initial setup and calibration process is made before is available to run experiments. This procedure applies to both the leader and following agents. The setup consists of the following (in the same order):
1) The MCU connects to the Wi-Fi network and the respective MQTT server located in the same local network.
2) The dead-band zone detection of the DC motor is performed. In this step, the motor speed is gradually increased until the train moves. It is important that the LED light at the back of the train is turned on since the measurement of the optical sensor is used to detect movement.
3) The camera calibration is performed. In this stage is important that the train has an object in front so it can measure its distance to the object. The train moves forward for about a second and compares the distances measurements to the object against the distance traveled detected by the camera. ***(WIP)***

At each stage, a message telling if the step was successful or not, is printed by the serial port. After all the steps are done the train is ready to run experiments!



### Performing an Experiment