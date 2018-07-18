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

## Usage

```
imageIMU [options]
  mode: 'qhd', 'hd', 'sd' or 'ir'
  visualization: 'image', 'cloud' or 'both'
  saving path: /home/username/dataset/
```

Example: `rosrun camera_imu imageIMU qhd image /home/user/dataset/`

## Key bindings

Image Windows:
- `ESC`, `q`: Quit.
- `b`: Save the RGB and depth image sequences.
- `e`: Stop saving.

