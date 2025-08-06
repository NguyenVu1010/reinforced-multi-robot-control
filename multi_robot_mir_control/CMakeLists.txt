cmake_minimum_required(VERSION 3.0.2)
project(multi_robot_mir_control) ## THAY BẰNG TÊN PACKAGE CỦA BẠN

## Tìm các package cần thiết để build.
## Thêm các package ROS mà bạn sử dụng ở đây.
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  gazebo_msgs
  tf
  message_generation
)

##=============================================================================
## Section 1: Build - Khai báo các file Service/Message
##=============================================================================

# Khai báo các file service bạn đã tạo trong thư mục 'srv'
add_service_files(
  FILES
  ProvidePath.srv
  # RequestPath_run.srv
  RequestPath.srv
  # Thêm các file .srv khác ở đây nếu có
)

# Khai báo các file message bạn đã tạo trong thư mục 'msg' (nếu có)
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Tạo ra các file mã nguồn (Python, C++, Lisp) từ các file .msg và .srv
## Đảm bảo rằng các dependencies của message/service được liệt kê ở đây
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

##=============================================================================
## Section 2: Catkin Package
##=============================================================================

# Khai báo các dependencies cần thiết khi build và chạy
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    rospy 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    gazebo_msgs 
    tf
    message_runtime
#  DEPENDS system_lib
)

##=============================================================================
## Section 3: Install
##=============================================================================

# Chỉ định các thư mục chứa file Python để catkin có thể tìm thấy
# và cài đặt chúng.
catkin_install_python(PROGRAMS
  # Liệt kê các file script chính trong thư mục 'scripts' (nếu có)
  # scripts/my_python_script.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Cài đặt các file launch, config, v.v.
install(DIRECTORY
  launch/
  config/
  # Thêm các thư mục khác bạn muốn cài đặt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Cài đặt các file Python trong thư mục src để có thể import
# Điều này rất quan trọng cho cấu trúc dự án của bạn
install(PROGRAMS
    train.py
    path_broadcaster.py
    read_weights.py
    run_robot.py
    run_training.py
    # Thêm các file thực thi khác ở thư mục gốc ở đây
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY
    core/
    env/
    models/
    utils/
    DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)