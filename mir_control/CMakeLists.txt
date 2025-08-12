cmake_minimum_required(VERSION 3.0.2)
project(mir_control)

##=============================================================================
## Section 0: Tìm kiếm các gói phụ thuộc (Dependencies)
##=============================================================================

# Tìm catkin và tất cả các component cần thiết cho cả Python và C++
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  gazebo_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

# Tìm thư viện Eigen3 cho C++
find_package(Eigen3 REQUIRED NO_MODULE)

##=============================================================================
## Section 1: Build - Khai báo các file Service/Message
##=============================================================================

# Khai báo các file service bạn đã tạo trong thư mục 'srv'
add_service_files(
  FILES
  ProvidePath.srv
  RequestPath.srv
)

# Khai báo các file message bạn đã tạo trong thư mục 'msg'
# Giả sử bạn có các file này
add_message_files(
  FILES
  Observation.msg
)

# Tạo ra các file mã nguồn (Python, C++, Lisp) từ các file .msg và .srv
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  gazebo_msgs
)

##=============================================================================
## Section 2: Catkin Package
##=============================================================================

# Khai báo các dependencies của package này
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
    rospy 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    gazebo_msgs 
    message_runtime
    tf2
    tf2_ros
#  DEPENDS Eigen3 # Không cần khai báo Eigen3 ở đây
)

##=============================================================================
## Section 3: Build - Cấu hình biên dịch
##=============================================================================

# Chỉ định các thư mục chứa file header cho trình biên dịch C++
include_directories(
  # Các header của Eigen phải được thêm vào
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# Khai báo file thực thi C++ cho observation node
add_executable(observation_calculator_node env/include/observation_calculator_node.cpp)

# Thêm dependency: Phải build xong các message trước khi build node này
add_dependencies(observation_calculator_node ${PROJECT_NAME}_generate_messages_cpp)

# Liên kết (link) node với các thư viện cần thiết
target_link_libraries(observation_calculator_node
  ${catkin_LIBRARIES}
)

# Thêm các file thực thi C++ khác ở đây nếu có
# add_executable(state_server_node src/state_server_node.cpp)
# add_dependencies(state_server_node ${${PROJECT_NAME}_MESSAGE_H_TARGETS})
# target_link_libraries(state_server_node ${catkin_LIBRARIES})

##=============================================================================
## Section 4: Install - Cài đặt các file
##=============================================================================

# Cài đặt các file Python trong thư mục src để có thể import
install(DIRECTORY
    core/
    env/
    models/
    utils/
    DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# Cài đặt các file thực thi Python
install(PROGRAMS
    train.py
    path_broadcaster.py
    read_weights.py
    run_robot.py
    run_training.py
    # Thêm các file thực thi khác ở thư mục gốc ở đây
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Cài đặt các file launch, config, v.v.
install(DIRECTORY
  launch/
  config/
  # Thêm các thư mục khác bạn muốn cài đặt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)