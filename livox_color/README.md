# LIVOX-COLOR-ROS
The livox color ros package is a tool for synchronizing livox lidar to generate color point cloud information. This node also subscribes to livox's /livox/lidar topic and camera's /hikrobot_camera/rgb topic to generate /livox/livox_color topic.
Detailed guidance reference:https://blog.csdn.net/weixin_41965898/article/details/117447600

# Install
```
mkdir -p ~/ws_livox_color/src
git clone https://github.com/luckyluckydadada/LIVOX_COLOR.git ~/ws_livox_color/src/livox_color
cd ~/ws_livox_color
catkin_make
```
# RUN
## run livox color fuision
```
source ./devel/setup.bash 
roslaunch livox_color color_livox.launch
```
# Tools
## 录制彩色点云topic为rosbag
```
rosbag record /livox/color_lidar
```
## 将录制的点云rosbag(PointCloud2)转为pcd(xyz)，并合并
```
source ./devel/setup.bash 
roslaunch livox_color bag2merge_pcd.launch \
input:=/home/$USER/rosbag/livox/PointCloud2bag/2021-06-11-15-10-57.bag \
output:=/home/$USER/rosbag/livox/PointCloud2bag/ \
threshold:=1
参数：
    input: bag的路径
    output： pcd的路径
    threshold： 帧数，3代表1s，目前彩色点云处理速率是1秒3帧；1代表不合并;0代表合并所有帧
解释：
    将11.bag(11代表这个bag文件含点云11帧)输出为3.pcd（3代表3帧合并一个pcd，既1秒）

```
## 将录制的彩色点云rosbag(PointCloud2)转为pcd(xyzrgb)，并合并
```
source ./devel/setup.bash 
roslaunch livox_color colorbag2merge_pcd.launch \
input:=/home/$USER/rosbag/livox/colorbag/fixed_scenes1/21.bag \
output:=/home/$USER/rosbag/livox/colorbag/fixed_scenes1/ \
threshold:=1
参数：
    input: bag的路径
    output： pcd的路径
    threshold： 帧数，3代表1s，目前彩色点云处理速率是1秒3帧；1代表不合并;0代表合并所有帧
解释：
    将11.bag(11代表这个bag文件含点云11帧)输出为3.pcd（3代表3帧合并一个pcd，既1秒）

```