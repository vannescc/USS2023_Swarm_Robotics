## Navigating the Robot's Directories
from the home directory you can locate the cloud9 directory, which is the default working directory if you're using cloud9 by typing "cd/var/lib/cloud9".
## Connecting to a network
To communicate with the Overlord the robot must be on the same wifi network as the Overlord. We opperate on "MU-Gaming" to check what network the robot is on type "iwconfig". if you're on the wrong network you can change networks by typing "connmanctl" then "scan wifi", then "services" to show all available networks, then to connect to a network type "connect [network]" the network would be listed in the format "wifi-......".
## Running "Movement.py"
To run all python code type "sudo python3 [FILENAME.py]" sudo is required to bypass certain permission restrictions. To run "Movement.py" you need to be in a tmux session, type "tmux" to create a session, to list all current sessions type "tmux ls", and then "tmux attach" to enter an existing session, to exit a session type "exit".
