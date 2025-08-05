#setup
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/

# clone mir_robot into the catkin workspace
git clone -b noetic https://github.com/DFKI-NI/mir_robot.git
#clone package multi_robot
git clone 
# use rosdep to install all dependencies (including ROS itself)
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep
sudo rosdep init
rosdep update --include-eol-distros
rosdep install --from-paths ./ -i -y --rosdistro noetic

# build all packages in the catkin workspace
source /opt/ros/noetic/setup.bash
catkin_init_workspace
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo

# chỉ train
  cd ~/catkin_ws
  catkin_make
  source ~/catkin_ws/devel/setup.bash
  roslaunch multi_mir_gazebo multi_robot_simulation.launch
  # mở terminal mới
  source ~/catkin_ws/devel/setup.bash
  roslaunch multi_robot_mir_control train.launch

# chỉ chạy 
  cd ~/catkin_ws
  catkin_make
  source ~/catkin_ws/devel/setup.bash
  roslaunch multi_mir_gazebo multi_robot_simulation.launch
  # mở terminal mới
  source ~/catkin_ws/devel/setup.bash
  roslaunch multi_robot_mir_control run.launch
#có thẻ thực hiện chỉnh sửa Map bằng graph_editor.py hoặc truy cập dashboard để chỉnh sửa

# chạy khởi đầu với dashboard
  cd ~/catkin_ws
  catkin_make
  source ~/catkin_ws/devel/setup.bash
  python3 src/multi_mir_robot/multi_robot_mir_control/multi_robot_dashboard/app.py
  # truy cập trình duyệt tại port vừa mở và thực hiện kết nối điều khiển



