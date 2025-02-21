LPRNet-OCR_ROS

OCR for car license plates using LPRNet in ROS

Detailed explanations will be added later.

Environment:
	•	Ubuntu 20.04
	•	ROS Noetic

How to Run: roslaunch ocr_ros ocr_ros.launch

This code is a modified version of Intel’s License Plate Recognition (LPRNet) to enable running it in a ROS environment.
For the original technology details, please refer to:
License Plate Recognition using LPRNet Model (Intel)

```
cd ~/catkin_ws/src
git clone https://github.com/jihoon7171/LPRNet-OCR_ROS.git
cd ..
catkin_make
```
execution command
```
roslaunch ocr_ros ocr_ros.launch
rosrun ocr_ros detect.py
```
![image](https://github.com/user-attachments/assets/8186dba8-6ad5-45dc-9e13-39daa9b17dd5)


Copyright Notice: 

This project is not the original creation of the LPRNet model. The original LPRNet model was developed by Intel.
I have only made modifications to adapt it for use in ROS (Robot Operating System).

For any copyright concerns, please refer to the original documentation linked above.

https://www.intel.co.kr/content/www/kr/ko/content-details/671328/license-plate-recognition-using-lprnet-model.html


### License
This project is licensed under the Apache License 2.0. See the [LICENSE](./LICENSE) file for details.
