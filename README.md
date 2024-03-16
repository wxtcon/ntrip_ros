
# NTRTP Client for ROS

**Authors Maintainers: Damon (Email: wxtcon@gmail.com)**

Have you ever found that your u-blox receiver just won't cooperate with your RTK dreams when using the [ublox_driver](https://github.com/HKUST-Aerial-Robotics/ublox_driver)?

Fear not, for **ntrip_client** is here to save the day! This handy tool is designed to bridge the gap between your u-blox receiver and NTRIP required to upload GGA messages, such as [Qianxun CORS](https://www.qxwz.com/products/findcm) and China Mobile CORS.

Here's what **ntrip_client** brings to the table:

1. GGA Message Generation: Subscribe and parse the ```/ublox_driver/receiver_pvt``` topic and craft GGA messages to feed  NTRIP casters.

2. RTCM Stream Relay: Once ntrip_client is up and running, it slurps up RTCM streams and forwards them to your u-blox receiver through a port of your choosing.   

3. No Extra Gadgets Required: Forget about fancy USB-to-TTL modules and other unnecessary accessories.   All you need is a computer and a u-blox receiver with a GNSS antenna (like the ublox_f9p), and you're good to go!
4. Reconnect if the network connection fails, ensuring uninterrupted communication.


## 1. Prerequisites
### 1.1 C++ 11 Compiler
This package requires some features of C++11.

## 1.2 ROS
Make sure your ROS version is above [ROS melodic](https://wiki.ros.org/melodic).

### 1.3 Eigen
The [Eigen 3.3.3](https://gitlab.com/libeigen/eigen/-/archive/3.3.3/eigen-3.3.3.zip) is adopted for matrix manipulation.

### 1.4 gnss_comm
This package requires [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm) for ROS message definitions and some utility functions. 
Follow [those instructions](https://github.com/HKUST-Aerial-Robotics/gnss_comm#1-prerequisites) to build the *gnss_comm* package.

### 1.5 ublox_driver
[ublox_driver](https://github.com/HKUST-Aerial-Robotics/ublox_driver) is used for receiver ublox messages from f9p module and get the PPP soultion.
Ntrip_client also need the ros topic from ublox_driver.

## 2. Build ntrip_client
Clone the repository to your catkin workspace (for example `~/catkin_ws/`):
```
cd ~/catkin_ws/src/
git clone https://github.com/wxtcon/ntrip_client.git
```
Then build the package with:
```
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
```
## 3. Run with you F9P module
1. First you need to configure RTCM-related information, which is located in the ```./config/ntrip_config.yaml``` file.

```
server_ip:     NTRIP caster IP ,such as 120.253.239.161(China mobile)
server_port:   NTRIP caster TCP Port, such as 8002. In general, different ports correspond to different geodetic coordinate systems, such as 8002 for WGS84
mountpoint:    NTRIP caster mount point, which relates your RTK device.
user:          user name. if you NTRIP caster needn't it, Please ignore this item or use the email name instead(such as RTK2go NTRIP Caster)
passwd:        password. if you NTRIP caster needn't it, Please ignore this item or use "none"(such as RTK2go NTRIP Caster)
transfer_port: TCM tranfer port, If you use ublox_driver, it should default to 3503.
transfer_ip:   RTCM tranfer IP. No modification is usually required if the ublox_driver is running on the same computer as the ntrip_client

```
2. Run ntrip_client
Make sure your computer has access to the Internet.
Then, Use the following command to run ntrip_client.

```
roslaunch ntrip_client start_ntrip_client.launch
```
If everything is OK, you should see the following message printed in your ubuntu terminal
```
[ INFO] [1710616230.433886318]: The TCP service is started, IP address: 127.0.0.1, Port: 3503
[ INFO] [1710616230.433960774]: Start listening the client...
```

3. Run ublox_driver
Ensure that ublox_driver is configured and rtcm reception is turned on.

Use the following command to run ublox_driver.
```
roslaunch ublox_driver ublox_driver.launch
```

ntrip_client will print the following message:
```
[ INFO] [1710616334.436246253]: Client access!--------
```
At this point, all communications are established.

4. Check the RTK solution states.
You can print the ```/ublox_driver/receiver_pvt``` topic to Check the RTK solution states.

```
rostopic echo /ublox_driver/receiver_pvt
```

If the item ```carr_soln``` of topic ```/ublox_driver/receiver_pvt``` turning to```2```,  which means RTK fixed solution.(```0``` means not RTK solution, and ```1``` means RTK float solution)

Now, enjoy it.

## 4. Notation
Due to the short development time, this is an **imperfect** program. We look forward to your valuable suggestions by submitting issues and other means.

## 5. Acknowledgements
In the process of development, we have referred to the excellent works of [ublox_driver](https://github.com/HKUST-Aerial-Robotics/ublox_driver), [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm), [NTRIP_ROS](https://github.com/Mil1ium/NTRIP_ROS.git), etc., and we would like to thank them all.




