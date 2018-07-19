# IMUKin2

## Maintainer

- [Zhixing Hou](https://sites.google.com/view/zhixing-hou) <<zxhou@njust.edu.cn>>, [The Intelligent Machine Research Lab](https://sites.google.com/view/huikonglab/home), Nanjing University of Science and Technology.


## Description

This is a simple program for reading and saving IMU (mpu6050) as well as Kinect v2 synchronously.

This is based on [Kinect2 Viewer](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_viewer) contributed by [Thiemo Wiedemeyer](https://ai.uni-bremen.de/team/thiemo_wiedemeyer).

## Dependencies

- ROS Kinetic
- OpenCV
- PCL

## Installation  
1. Install [libfreenect2](https://github.com/OpenKinect/libfreenect2)  
2. Install [iai_kinect2](https://github.com/code-iai/iai_kinect2)  
3. Clone this repository into your catkin workspace and build it.    
```
cd ~/catkin_ws/src/
git clone https://github.com/zxhou/IMUKin2.git
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"
```
## Usage
1. Check the serial port devices.`ls -l /dev/tty*`  
2. For mpu6050, ttyUSB0 is available. `sudo chmod 666 /dev/ttyUSB0` (every time you boot the machine)  
3. Run the program
```
imageIMU [options]
  mode: 'qhd', 'hd', 'sd' or 'ir'
  visualization: 'image', 'cloud' or 'both'
  saving path: ~/dataset/
```
Example: `rosrun imukin2 imukin2 qhd image ~/dataset/`

## Key bindings

Image Windows:
- `ESC`, `q`: Quit.
- `b`: Save the RGB and depth image sequences into `~/dataset/color` and `~/dataset/depth` respectively. Save IMU data into `~/dataset/imuData.txt`.
- `e`: Stop saving.

