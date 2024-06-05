# PGMF
---
## **Perpendicular-based 3D Gaussian-Uniform Mappoint Filter**
---
**04 June 2024**: An initial version of PGMF, which supports monocular camera + IMU.

![PGMF-VINS.svg](PGMF-VINS.svg)

PGMF is a **monocular VIO SLAM algorithm**. This algorithm proposes a convergence strategy for mappoints which can effectively improve the quality of the map while ensuring state-of-the-art  accuracy of localization. 



### **1. Prerequisites**
---
**1.1  Ubuntu and ROS :** Ubuntu 20.04 and ROS Noetic.

**1.2  Eigen :** Eigen 3.3.7.

**1.3  Ceres :**  Follow [**Ceres installation**](http://ceres-solver.org/installation.html) to install Ceres for version 1.14.0 and remember to **sudo make install**.

**1.4  Sophus:**  Sophus 1.22.10. You can get from [**sophus website**](https://github.com/strasdat/Sophus).



### 2. **Build PGMF program**
---
Clone the repository and catkin_make:

```
git clone https://github.com/WenqingDeng/PGMF.git
cd PGMF/
catkin_make
source ./devel/setup.bash
```



### 3. **Visual-Inertial Odometry on Public datasets**
---
Download [**EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).**

You can also test on [**ETH-asl cla dataset**](http://robotics.ethz.ch/~asl-datasets/maplab/multi_session_mapping_CLA/bags/), but we do not provide parameters settings, so you have to adjust the parameters in config file.

Open two terminals, launch the VIO, and play the bag file respectively. For example

```
roslaunch demo euroc.launch 
rosbag play YOUR_PATH_TO_DATASET/MH_01_easy.bag 
```



### 4. Run on your device
---
4.1 Ensure that the prerequisites completed properly.

4.2 Change to your topic name in the config file.

4.3 You can calibrate the extrinsics between camera and imu, with any tools you like. Then just write the parameters in the config file in the right format.

4.4 Other parameter: make appropriate adjustments of other parameters in the config file to achieve better results.




### 5. **Related Papers**
---
The paper is currently under review of **PG2024**.




### 6. **Licence**
---
The source code is released under **[GPLv3](http://www.gnu.org/licenses/)** license.