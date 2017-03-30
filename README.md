# ros_ocare_nav
使用ROS + Gazebo 來進行 move_base實作

---
# Required

## 1. [WiringPi](http://wiringpi.com/)

```
$ git clone git://git.drogon.net/wiringPi
$ cd ~/wiringPi
$ ./build
```

## 2. [libmodbus](http://libmodbus.org/)
```
$ sudo apt-get update
$ sudo apt-get install "libmodbus*" -y
```

## 3. [multimaster_fkie](http://wiki.ros.org/multimaster_fkie)
```
$ sudo apt-get update
$ sudo apt-get install sudo apt-get install "ros-kinetic-multimaster-*" -
```

## 3. [joy](http://wiki.ros.org/joy)
```
$ sudo apt-get update
$ sudo apt-get install "ros-kinetic-joy-*" -y
```

---
## 2016/04/05
完成Git版本管理建立
## 2016/04/05
目前已經能定位自己位置使用ACML and laser_matcher or Gmapping and laser_matcher
## 2016/09/03
完成比賽
