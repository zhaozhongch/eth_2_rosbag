This package is just used to tansform [eth-3d](https://www.eth3d.net/slam_documentation) dataset to ros message
# Install
In your  folder, clone this package
```
cd catkin_ws/src
git clone https://github.com/zhaozhongch/eth_2_rosbag.git
cd ..
catkin_make
```
# Use
Change the `dataset_address` in `generate_eth_rosbag` to the address you put the dataset
```
cd catkin_ws/
source devel/setup.bash
roslaunch eth_2_rosbag generate_eth_rosbag.launch
```
You'll get 5 topics
```
/depth
/groundtruth
/image0
/imu0
/rgb_image0
/rgb_image1
```
where `image0` is the gray image. Code should be straigtforward to understand.  
You should put the `eth_config` folder into your `VINS FUSION`'s config folder.