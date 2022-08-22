# AIロボット入門のためのTurtlebot2モデル

## 概要

## インストール

```
sudo apt -y install ros-foxy-gazebo-ros ros-foxy-camera-info-manager
sudo apt -y install ros-foxy-realsense2-description
sudo apt -y install ros-foxy-diagnostic-updater ros-foxy-ecl-linear-algebra ros-foxy-kobuki-core ros-foxy-xacro ros-foxy-joint-state-publisher
sudo apt -y install ros-foxy-navigation2 ros-foxy-nav2-bringup
sudo apt -y install ros-foxy-urg-node
sudo apt -y install ros-foxy-tf-transformations

cd ~/airobot_ws/src
git clone https://github.com/kobuki-base/kobuki_ros_interfaces
git clone -b foxy-devel https://github.com/pal-robotics/realsense_gazebo_plugin
git clone https://github.com/rt-net/crane_plus
git clone https://github.com/AI-Robot-Book/kobuki_ros_airobotbook

rosdep install -r -y -i --from-paths .

cd ~/airobot_ws
colcon build
source install/setup.bash
```

### デバイスファイルのルールの設定

```
sudo cp ~/airobot_ws/src/turtlebot2_airobotbook/rules.d/* /etc/udev/rules.d
```
KobukiとCRANE+ V2は同じFTDIのドライバを使っているので，個別の機器に合わせたルールの書き換えが必要．うまく設定できれば，以下のようなデバイスファイル（のシンボリックリンク）が作られる．

- Kobuki → `/dev/ttyUSB_kobuki`
- CRANE+ V2 → `/dev/ttyUSB_craneplus`

## 実行

### Kobukiのモデルの確認
```
ros2 launch kobuki_description robot_description.launch.py rviz:=True
```
### Turtlebot2のシミュレーションの基本
```
ros2 launch turtlebot2_airobotbook turtlebot2_gazebo.launch.py 
```
### Turtlebot2のシミュレーションとRVizの併用
```
ros2 launch turtlebot2_airobotbook turtlebot2_gazebo_rviz.launch.py 
```
### Turtlebot2のシミュレーションで地図作成
```
ros2 launch turtlebot2_airobotbook turtlebot2_gazebo_slam.launch.py 
```
### Turtlebot2のシミュレーションでナビゲーション
```
 ros2 launch turtlebot2_airobotbook turtlebot2_gazebo_nav2.launch.py 
```
### Turtlebot2 + CRANE+ V2 Realsense D435 のシミュレーションの基本
```
ros2 launch turtlebot2_airobotbook turtlebot2_crane_gazebo.launch.py 
```

### Turtlebot2 + CRANE+ V2 Realsense D435 のシミュレーションとRVizの併用
```
ros2 launch turtlebot2_airobotbook turtlebot2_crane_gazebo_rviz.launch.py 
```


