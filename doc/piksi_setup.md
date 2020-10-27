# How to setup Piksi on the Actor2 vehicle (running kinetic and python2)

## How to install ROS side

Clone the `piksi_ethz_ros` package, and checkout the correct version that still supports kinetic:

```sh
cd path/to/workspace/src
git clone https://github.com/ethz-asl/ethz_piksi_ros.git
cd ethz_piksi_ros
git checkout v1.11.0
```

Run the install script. Normally this is `piksi_ethz_ros/piksi_multi_rtk_ros/install/install_piski_multi.sh`. However, this script appears to have broken since October 2019. Instead use the modified version of the script in this package:

```sh
./path/to/workspace/src/actor2_suport/scripts/install_piksi_multi.sh
```

Now (after building and sourcind the workspace), the piksi launch file should work:

```sh
roslaunch actor2_support gps.launch
```
