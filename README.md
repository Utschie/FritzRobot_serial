# FritzRobot_serial
### 2023-10-25 update
The data is read from the USBVCom. However, a very bad thing is that because now the output of the USBVCom is currently in the form of a uint8_t array, the array has to be converted to a string in the read by first extracting segments from the vector, and then converting the string to a float. 

**In the future It should be changed to a more efficient and space-saving way of transferring the data.**
### 2023-10-24 update
read robot's state from USBVCom and showing on the screen. Making sure that the USB permission has released.
```
sudo chmod 777 /dev/ttyACM0
```