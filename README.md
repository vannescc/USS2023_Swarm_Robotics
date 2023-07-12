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



This file contains the contents of the /home/pi/Swarm_Robots_USS2023/apriltag-master/python folder
The Overlord is responsible for locating the robot in relation to the boundry box, communicating with the robot, and all computation
The Overlord can tell the robot how fast and how for how long to drive straight, and can tell it to turn an angle
It can also instruct the robot to drive along a circle, but this function is not currently functional
There are several verisons of several of the programs, the python file with the largest version number is the most recent
