# Project_AC
## Haptic finger connection
## Haptic finger:

Connect the micro-usb to the port close to HTMI port (the one in the middle) of he raspberryppi device.
In the wired connection setting of the sensor (Linux Network settings), set raspberryppi IPV6 to "Disable" and the IPV4 to "Linnk-Local".
Open now different terminals and do the folowing instructions.

### firewall disabling
If a firewall is enabled you should disable it in one of the following ways:
```
sudo ufw disable #disable permanently the firewall
```
or temporarly (you have to do the procedure again if you restart the pc):
```
sudo iptables -L
sudo iptables -F   # Flush all rules
sudo iptables -P INPUT ACCEPT   # Allow all incoming traffic
sudo iptables -P FORWARD ACCEPT # Allow all forwarded traffic
```

### Terminal 1 -  client terminal 
ssh is a pront that allows you to connect remotely to another pc. In this case we connect to the raspberrypi, so we can control it from the terminal in our pc. 
```
ssh pi@raspberrypi.local   # (password:  2880)
sudo nano client.py
```
the nano prompt allows you to edit the file that comes after the prompt, while sudo gives you the adminstrator permissions required.

### Terminal 2
Get the raspberrypi ip adress:
```
hostname -I
```
### Terminal 1
Edit the raspberrypi ip adress which you already got in the "hostmame -I" output, it should start with "169.254." edit that ip adress then "ctr+o" to save and "ctr+x" to exit

### Terminal 3 - server terminal
Locate the server_sersor.py then edit the raspberrypi ip adress (same as client.py)
```
sudo nano server.py
```

### Terminal 4
```
roscore
```

### Terminal 3
Now you can run the server first by:
```
python3 server.py
```

### Terminal 1
And run the client on the pi terminal
```
python3 client.py
```

Everything should be working alright now and you can check by rostopic echo that if the sensor image is being published properly.
```
rostopic list  # the topic published should be /fing_camera/color/image_raw
rostopic echo .............
```

For seeeing the image you can also do:
```
rqt_image_viewer #and select the topic to print
```
Troubleshoot:
- If you cannot ssh (or cannot ping raspberrypi.local), sometime a restart or reconnecting the sensor will solve the prblem.

## Data collection with the haptic finger

First of all make sure that the haptic finger is connect and the topic is published. You must see the camera frame in your screen.
For collecting the data you have to launch the controller and movit as first thing. In particular you have to execute these commands:
```
cd tactile_control_ale
source devel/setup.bash
roslaunch franka_teleoperation data_collection.launch
```
The robot ip is passed as a default argument, change it in the script if you have to. The controller included is 
```
franka_control.launch
```
while the moveit script is 
```
panda_moveit.launch
```
from the panda_moveit_config package.

Then in a new terminal you have to run the data_collection_script in which is defined the trajectory you want to send to the robot, read and saves the data etc.. In my case:
```
cd /home/alessandro/tactile_control_ale/src/data_collection/franka_teleoperation/src
python3 data_collection_ALE
```
you can modify this script as you want depending on your aims.
An ideal Dataset dimension is about 1000 samples taken at a frequency of 30 Hz.

## Launching the pipeline

For executing the pipeline (updated to 17 february) you have to launch 3 files in the order I will mention them. But first of all you have to create a catkin workspace and install Franka_ros (0.8.0) and a compatible version of _libfranka_.
Essentialy we will use a modified version of cartesian_impedance_example_controller for communicate with the robot.
Let's see now the proceedure for launching the pipeline:

1. launch the stem localization predictor first and wait for the model to warm up.
    ```
   stem_pose_predictor.py
    ```
2. lunch the optmizer which generate the trajectory to send to the robot
