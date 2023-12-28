# FritzRobot_serial


## Description
 There are 3 nodes in this pkg. The nodes "downstream" and "upstream" communicate with the MCU. And the pose_estimation is to estimate the 2D/3D orientation of the robot.
##
### 2023-12-28 update
changed msg's type Twist to TwistStamped
### 2023-11-03 update
added launch file to start the "upstream", "downstream" and "pose_estimation" nodes.
### 2023-10-31 update

added downstream communication node to send transfer the msgs from /cmd_vel to MCU.


### 2023-10-30 update
added a new node estimating the pose of the robot. Since the daily scenario is 2D plane，so the z is set to be constant, which can help decrease the estimates‘ jitter. In the video is the pose estimation in 3D scenario.
<!--
<video width="320" height="240" controls>
    <source src="media/VID_20231030_192910.mp4" type="video/mp4">
</video>
-->

https://user-images.githubusercontent.com/33782458/279166788-9cf906c9-b439-4dda-a0e9-c7a8e0e92db5.mp4

### 2023-10-27 update
when reading the USB input buffer, if the data size is wrong, flush the input buffer. Seems to work for the segmentation fault.

### 2023-10-26 update
defined Wheelspeed.msg, implemented driver node to read from USB and publish to topics /chassis/imu /chassis/vel /chassis/wheelspeed. While some bugs exists,showing segmentation fault (core dumped).

### 2023-10-25 update
The data is read from the USBVCom. However, a very bad thing is that because now the output of the USBVCom is currently in the form of a uint8_t array, the array has to be converted to a string in the read by first extracting segments from the vector, and then converting the string to a float. 

**In the future It should be changed to a more efficient and space-saving way of transferring the data.**
### 2023-10-24 update
read robot's state from USBVCom and showing on the screen. Making sure that the USB permission has released.
```
sudo chmod 777 /dev/ttyACM0
```