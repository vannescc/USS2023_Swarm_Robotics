# USS2023_Swarm_Robotics
This project aimed to develop an routine for a robot to automatically charactorize it's own movement
## Overlord
The Overlord is responsible for locating the robot in relation to the boundry box, communicating with the robot, and all computation. 
## Robot
The robot receives instructions from the Overlord and can collect data from the IR sensor mounted on the front of the robot
## Running the Project
### Connecting to the BBBlue
I connect to the BeagleBone Blue in 3 ways:
1. Connect the BBBlue with a USB cable. In a PuTTy window, go to Serial connection and put in your COM port. more detailed instructions [here](https://static.packt-cdn.com/downloads/BeagleBoneRoboticProjectsSecondEdition_ColorImages.pdf).
2. When the BBBlue is connected via micro USB the you can use the [cloud9](https://beagleboard.org/support/bone101) IDE. This is how I chose to program the BBBlue.
3. Connect to the BBBlue's wifi. In a PuTTy window, select SSH as your connection type and type the IP address of your device (the default is 192.168.8.2). This is how I run the robot as it is the only wireless method of communication in this list.
### Finding the IP of your BBBlue
In the terminal type "ifconfig" to obtain a list of all IPs the BBBlue has with their corresponding connection types. When communicating with the BBBlue over a serial connection use the IP listed under "usb0", if connecting over the BBBlue's wifi use the IP under "SoftAp0"
###

### BBBlue pinout
![image](https://github.com/vannescc/USS2023_Swarm_Robotics/assets/120139813/50d27df0-8a13-4871-9a1d-82a0015406ed)
## IR Distance Sensor SHARP 0A41SK [(Datasheet)](https://www.pololu.com/file/0J713/GP2Y0A41SK0F.pdf)
I use the rcpy.adc library to read data from the IR sensor. This library returns an integer representation of the voltage passed by the IR sensor. This sensor must be calibrated before obtaining a distance reading from the IR sensor, I calibrated my sensor using an implimentation of [these instructions](https://aleksandarhaber.com/noise-reduction-and-calibration-of-distance-sensors-sharp-infrared-sensors/). Below is a graph of the estimation of the IR charectorization using least squares method.
![IR Integer Reading vs Distance](https://github.com/vannescc/USS2023_Swarm_Robotics/assets/120139813/560c7c6b-457c-45af-a7fb-a5165d806c5f)

the function to estimate distance based on the integer reading from the IR sensor is given by:

![image](https://github.com/vannescc/USS2023_Swarm_Robotics/assets/120139813/96ad3f36-62e1-458a-a11f-1682bac367e0)

where k1 = 62838.53689731416 and k2 = -1.3276711025926407

## IR Distance Sensor Hookup
![image](https://github.com/vannescc/USS2023_Swarm_Robotics/assets/120139813/5e28eb07-f63b-4058-80f8-a7f3b17571e8)

The IR Sensor is connected to the BBBlue on the robot via a breadboard located at the back of the robot. The wires that are plugged into ports on the beaglebone blue are wraped around 22 guage wires to have better contact with the breaboard. The sensor output is wired with a voltage divider of 20kohms.
