# PX4

## QGC地面站
``` shell
#Wifi数传通讯设置
MAV_1_CONFIG= TELEM_2 #设置TELEM_2为数传输出口
MAV_1_MODE = Onboard
SER_TEL2_BAUD = 921600 #设置波特率

#室内定点参数
EKF2_AID_MASK 24 #（不选择GPS，选择vision position fusion和vision yaw fusion）
EKF2_HGT_MODE Vision
MAV_ODOM_LP 1

#速度设置
MPC_XY_VEL_MAX 0.5
MPC_Z_VEL_MAX_DN 0.5
MPC_Z_VEL_MAX_UP 0.5

#PX4默认PID
MPC_XY_VEL_P_ACC 1.8
MPC_XY_VEL_I_ACC 0.6
MPC_XY_VEL_D_ACC 0.2

MPC_Z_VEL_P_ACC 4
MPC_Z_VEL_I_ACC 0.6
MPC_Z_VEL_D_ACC 0

MPC_XY_P 1.20
MPC_Z_P 1.20
```

## PX4 飞控稳定调试

- 首先在自稳模式下将PID参数调稳定，调试`Multicopter Rate Control`菜单下`MC_ROLLRATE_P`, `MC_ROLLRATE_I`, `MC_ROLLRATE_D`,`MC_PITCHRATE_P`,`MC_PITCHRATE_I`,`MC_PITCHRATE_D`这六个参数即可
- 自稳PID调试完成后，此时飞行会发现飞行器总是会朝某个方向“倾斜”飞行，此时最好先将机体各部位都固定稳定，中心最好在机体中心（电池位置最好也固定，不然电池的拆卸也是影响重心的一个要点）。然后调试`Sensors`菜单下的`SENS_BOARD_X_OFF`和`SENS_BOARD_Y_OFF`两个参数，最完美的状态是调试到roll和pitch不总是朝一个方向飞行，只会随机朝某个方向缓慢飞行
- 随后切换到offboard模式进行定位调试，如果设置指定高度后飞行器一直飞行不到指定高度，请增大`Multicopter Position Control`菜单下的`MPC_THR_HOVER`和`MPC_Z_P`参数
- -   随后调试定位的参数，`Multicopter Position Control`菜单下的`MPC_XY_P`,`MPC_XY_TRAJ_P`,`MPC_XY_VEL_D_ACC`,`MPC_XY_VEL_I_ACC`,`MPC_XY_VEL_P_ACC`这几个参数，注意增大其中`MPC_XY_VEL_I_ACC`参数对减小偏移有显著效果

## MAVESP8266

``` shell
#下载固件
https://firmware.ardupilot.org/Tools/MAVESP8266/latest/

#初始账户：ArduPilot
#初始密码：ardupilot

#连接上ESP8266后，打开192.168.4.1，连接实验室Wifi
```

# MAVROS

## 安装MAVROS
``` shell
sudo apt-get install ros-<ros_vision>-mavros ros-<ros_vision>-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/
	 wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/
	 install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
#若wget失败，可直接下载sh文件运行
```

## MAVROS测试
``` shell
#连接Wifi数传到飞控
#查看Wifi数传IP地址

#设置mavros参数
roscd mavros/launch/
sudo vim px4.launch
#fcu_url  udp://:14550@192.168.31.33:12345 //33为数传地址
#gcs_url  udp://:14556@192.168.31.27 //27为qgc主机地址（可以不设置）
#运行mavros
roslaunch mavros px4.launch 

#运行rostopic，查看状态
rostopic echo /mavros/state
#注意，Wifi数传一次只能连接一个终端，连接mavros后就不能连接QGC地面站

#如果显示数据异常，修改以下信息
roscd mavros/launch
sudo vim px4_config.yaml
#将timesync_rate: 10.0 改为timesync_rate: 0.0
```

# 动捕设置

## 安装VRPN
``` shell
sudo apt-get install ros-<your ros vision>-vrpn*
```

## 运行VRPN
``` shell
roslaunch vrpn_client_ros sample.launch  server：=192.168.31.128
#IP地址为动捕主机IP地址
```

# 实验平台操作

## 位置控制

``` shell 
#修改system_init.launch 
#将参数fcu_url修改为当前无人机的ip地址
#将参数server修改为动捕的ip地址
#将参数gcs_url修改为运行qgc主机的ip地址

#完成上述操作，运行launch
roslaunch experiments  system_init.launch 

#新建终端
rostopic echo /mavros/state
#检查是否能切换到位置控制模式，可以切换则进行下一步操作

#打开position.txt，将坐标点复制到文件中

#运行position_control节点
rosrun experiments position_control 
#检查读取的坐标点是否有误，无误则进行下一步

#无人机解锁，切换到offboard模式，无人机自动起飞，执行完任务后在任务终点悬停
```