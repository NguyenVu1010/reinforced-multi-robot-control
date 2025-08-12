#include <ros/ros.h>
#include <vector>
#include <string>
#include <cmath>
#include <thread>
#include <mutex>
#include <numeric>
#include <algorithm>
#include <limits>

// Core ROS messages
#include <sensor_msgs/LaserScan.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/UInt16.h"
// TF2 for transformations
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Eigen for linear algebra
#include <Eigen/Dense>

// Custom message
#include "mir_control/Observation.h" // THAY BẰNG TÊN PACKAGE CỦA BẠN

// Sử dụng float cho hiệu năng thay vì double
using Vector2f = Eigen::Vector2f;
using Matrix2f = Eigen::Matrix2f;
using PointsVector = std::vector<Vector2f, Eigen::aligned_allocator<Vector2f>>;
using OtherStatesMap = std::map<std::string, std::pair<Vector2f, Vector2f>>;

class ObservationCalculatorNode
{
public:
    ObservationCalculatorNode() : nh_("~")
    {
        // Lấy tham số
        nh_.param<std::string>("robot_name", robot_name_, "robot1");
        std::string all_robots_str;
        nh_.param<std::string>("all_robot_names", all_robots_str, "robot1,robot2,robot3");
        
        std::stringstream ss(all_robots_str);
        std::string item;
        while (std::getline(ss, item, ',')) {
            if (item != robot_name_) {
                other_robot_names_.push_back(item);
            }
        }
        
        num_lidar_clusters_ = 6;
        last_nearest_idx_ = 0;
        total_path_length_ = 0.0f;
        current_timestep_ = 0;
        nh_.param<int>("/max_steps_per_episode", max_steps_, 4000);

        // Publishers
        obs_pub_ = nh_.advertise<mir_control::Observation>("/" + robot_name_ + "/rl_observation", 5);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/debug/" + robot_name_ + "/future_points_marker", 1);
        nearest_pub = nh_.advertise<std_msgs::UInt16>("/" + robot_name_ + "/nearest_idx", 1);
        // Subscribers
        scan_sub_ = nh_.subscribe("/" + robot_name_ + "/scan", 1, &ObservationCalculatorNode::lidarCallback, this);
        path_sub_ = nh_.subscribe("/paths/" + robot_name_ + "/planned_path", 1, &ObservationCalculatorNode::pathCallback, this);
        model_states_sub_ = nh_.subscribe("/gazebo/model_states", 1, &ObservationCalculatorNode::modelStatesCallback, this);
        
        ROS_INFO("[%s] ObservationCalculatorNode (C++) đã khởi tạo.", robot_name_.c_str());
    }

    void run()
    {
        ros::Rate rate(20); 
        while (ros::ok())
        {
            _calculate_and_publish();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    std::string robot_name_;
    std::vector<std::string> other_robot_names_;
    
    ros::Publisher obs_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher nearest_pub;
    ros::Subscriber scan_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber model_states_sub_;
    std::mutex data_mutex_;
    sensor_msgs::LaserScan::ConstPtr latest_lidar_;
    gazebo_msgs::ModelStates::ConstPtr latest_model_states_;
    
    // State nội bộ
    PointsVector path_;
    float total_path_length_;
    int last_nearest_idx_;
    int current_timestep_;
    int max_steps_;
    int num_lidar_clusters_;
    Vector2f current_pos_;
    float current_yaw_;
    Vector2f current_velocity_;

    // Callbacks
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_); latest_lidar_ = msg;
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (msg && msg->poses.size() > 1) {
            path_.resize(msg->poses.size());
            for (size_t i = 0; i < msg->poses.size(); ++i) {
                path_[i] << msg->poses[i].pose.position.x, msg->poses[i].pose.position.y;
            }
            total_path_length_ = 0.0f;
            for (size_t i = 0; i < path_.size() - 1; ++i) {
                total_path_length_ += (path_[i+1] - path_[i]).norm();
            }
            last_nearest_idx_ = 0;
            current_timestep_ = 0;
        } else {
            path_.clear(); total_path_length_ = 0.0f;
        }
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_); latest_model_states_ = msg;
    }

    void _calculate_and_publish()
    {
        sensor_msgs::LaserScan::ConstPtr lidar_snapshot;
        gazebo_msgs::ModelStates::ConstPtr model_states_snapshot;
        PointsVector path_snapshot;
        int last_idx_snapshot;
        float total_path_len_snapshot;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!latest_lidar_ || !latest_model_states_ || path_.empty()) return;
            lidar_snapshot = latest_lidar_;
            model_states_snapshot = latest_model_states_;
            path_snapshot = path_;
            last_idx_snapshot = last_nearest_idx_;
            total_path_len_snapshot = total_path_length_;
            current_timestep_++;
        }
        
        OtherStatesMap other_states;
        auto it = std::find(model_states_snapshot->name.begin(), model_states_snapshot->name.end(), robot_name_);
        if (it == model_states_snapshot->name.end()) return;
        int my_idx = std::distance(model_states_snapshot->name.begin(), it);

        const auto& my_pose_msg = model_states_snapshot->pose[my_idx];
        const auto& my_twist_msg = model_states_snapshot->twist[my_idx];
        tf2::Quaternion q(my_pose_msg.orientation.x, my_pose_msg.orientation.y, my_pose_msg.orientation.z, my_pose_msg.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        current_pos_ << my_pose_msg.position.x, my_pose_msg.position.y;
        current_yaw_ = static_cast<float>(yaw);
        current_velocity_ << my_twist_msg.linear.x, my_twist_msg.angular.z;

        for (const auto& name : other_robot_names_) {
            auto other_it = std::find(model_states_snapshot->name.begin(), model_states_snapshot->name.end(), name);
            if (other_it != model_states_snapshot->name.end()) {
                int other_idx = std::distance(model_states_snapshot->name.begin(), other_it);
                other_states[name] = {
                    Vector2f(model_states_snapshot->pose[other_idx].position.x, model_states_snapshot->pose[other_idx].position.y),
                    Vector2f(model_states_snapshot->twist[other_idx].linear.x, model_states_snapshot->twist[other_idx].linear.y)
                };
            }
        }

        auto path_props = _get_path_properties(current_pos_, current_yaw_, current_velocity_, path_snapshot, last_idx_snapshot);
        float ep = std::get<0>(path_props);
        float alpha = std::get<1>(path_props);
        std::vector<float> future_points_10d = std::get<3>(path_props);

        float path_approach_speed = _calculate_path_approach_speed(current_pos_, current_yaw_, current_velocity_, path_snapshot, last_nearest_idx_);
        std::vector<float> obstacle_info = _get_lidar_obstacle_info(lidar_snapshot);
        
        Vector2f rel_pos, rel_vel;
        std::tie(rel_pos, rel_vel) = _get_nearest_robot_full_info(current_pos_, current_yaw_, current_velocity_, other_states);
        
        float dist_to_goal_on_path = _calculate_remaining_path_distance(path_snapshot, last_nearest_idx_, total_path_len_snapshot);
        float normalized_timestep = static_cast<float>(current_timestep_) / max_steps_;

        mir_control::Observation obs_msg;
        obs_msg.data = {ep, alpha, current_velocity_.x(), current_velocity_.y(), path_approach_speed, normalized_timestep};
        obs_msg.data.insert(obs_msg.data.end(), obstacle_info.begin(), obstacle_info.end());
        obs_msg.data.insert(obs_msg.data.end(), future_points_10d.begin(), future_points_10d.end());
        obs_msg.data.push_back(rel_pos.x()); obs_msg.data.push_back(rel_pos.y());
        obs_msg.data.push_back(rel_vel.x()); obs_msg.data.push_back(rel_vel.y());
        obs_msg.data.push_back(dist_to_goal_on_path);

        obs_pub_.publish(obs_msg);
        std_msgs::UInt16 nearest_idx_msg;
        nearest_idx_msg.data = last_nearest_idx_;
        nearest_pub.publish(nearest_idx_msg);
    }
    
    std::tuple<float, float, bool, std::vector<float>> _get_path_properties(const Vector2f& robot_pos, float robot_yaw, const Vector2f& robot_vel, const PointsVector& path, int last_idx, int num_future_points = 5, int search_window = 800)
    {
        if (path.size() < 2) return {0.0f, 0.0f, true, std::vector<float>(num_future_points * 2, 0.0f)};
        int start_search = std::max(0, last_idx - search_window / 2);
        int end_search = std::min((int)path.size(), last_idx + search_window / 2);
        int nearest_idx = start_search;
        float min_dist_sq = std::numeric_limits<float>::max();
        for (int i = start_search; i < end_search; ++i) {
            float d_sq = (path[i] - robot_pos).squaredNorm();
            if (d_sq < min_dist_sq) { min_dist_sq = d_sq; nearest_idx = i; }
        }
        { std::lock_guard<std::mutex> lock(data_mutex_); last_nearest_idx_ = nearest_idx; }
        const Vector2f& p1 = path[nearest_idx];
        const Vector2f& p2 = path[std::min(nearest_idx + 1, (int)path.size() - 1)];
        Vector2f vec_path_to_robot = robot_pos - p1; Vector2f path_tangent = p2 - p1;
        Vector2f path_normal(-path_tangent.y(), path_tangent.x());
        float ep = -vec_path_to_robot.dot(path_normal) / (path_normal.norm() + 1e-6f);
        float linear_v = robot_vel.x();
        float look_ahead_dist = std::min(1.2f, std::max(0.4f, linear_v * 1.0f));
        int target_idx_for_alpha = nearest_idx;
        while (target_idx_for_alpha < path.size() - 1 && (path[target_idx_for_alpha] - path[nearest_idx]).norm() < look_ahead_dist) {
            target_idx_for_alpha++;
        }
        float angle_to_target = std::atan2(path[target_idx_for_alpha].y() - robot_pos.y(), path[target_idx_for_alpha].x() - robot_pos.x());
        float alpha = angle_to_target - robot_yaw;
        alpha = std::atan2(std::sin(alpha), std::cos(alpha));
        std::vector<float> flat_future_points_robot;
        flat_future_points_robot.reserve(num_future_points * 2);
        PointsVector future_points_world;
        Matrix2f rotation_matrix;
        rotation_matrix << std::cos(-robot_yaw), -std::sin(-robot_yaw), std::sin(-robot_yaw), std::cos(-robot_yaw);
        for (int i = 0; i < num_future_points; ++i) {
            float target_dist = (i + 1) * 0.5f;
            int temp_idx = nearest_idx;
            float dist_traveled = 0.0f;
            while (temp_idx < path.size() - 1 && dist_traveled < target_dist) {
                dist_traveled += (path[temp_idx+1] - path[temp_idx]).norm();
                temp_idx++;
            }
            const Vector2f& point_world = path[std::min(temp_idx, (int)path.size()-1)];
            future_points_world.push_back(point_world);
            Vector2f point_robot = rotation_matrix * (point_world - robot_pos);
            flat_future_points_robot.push_back(point_robot.x());
            flat_future_points_robot.push_back(point_robot.y());
        }
        _publish_future_points_marker(future_points_world, robot_name_);
        bool goal_reached = (robot_pos - path.back()).norm() < 0.3f;
        return {ep, alpha, goal_reached, flat_future_points_robot};
    }

    float _calculate_path_approach_speed(const Vector2f& robot_pos, float robot_yaw, const Vector2f& robot_vel, const PointsVector& path, int nearest_idx) {
        if (path.empty() || nearest_idx >= path.size()) return 0.0f;
        Vector2f vec_path_to_robot = robot_pos - path[nearest_idx];
        float norm = vec_path_to_robot.norm();
        if (norm < 1e-4f) return 0.0f;
        float linear_v = robot_vel.x();
        Vector2f robot_vel_vec(linear_v * std::cos(robot_yaw), linear_v * std::sin(robot_yaw));
        return std::tanh(robot_vel_vec.dot(vec_path_to_robot) / (norm + 1e-9f));
    }

    std::vector<float> _get_lidar_obstacle_info(const sensor_msgs::LaserScan::ConstPtr& lidar_data) {
        const float max_dist = 15.0f;
        if (!lidar_data || lidar_data->ranges.empty()) { return std::vector<float>(num_lidar_clusters_, max_dist); }
        std::vector<float> ranges = lidar_data->ranges;
        for (float& r : ranges) { if (std::isinf(r) || std::isnan(r)) { r = max_dist; }}
        int num_rays = ranges.size();
        int sector_size = num_rays / num_lidar_clusters_;
        std::vector<float> obstacle_info;
        obstacle_info.reserve(num_lidar_clusters_);
        for (int i = 0; i < num_lidar_clusters_; ++i) {
            auto start_it = ranges.begin() + i * sector_size;
            auto end_it = ranges.begin() + (i + 1) * sector_size;
            if (start_it < end_it) {
                obstacle_info.push_back(*std::min_element(start_it, end_it));
            }
        }
        return obstacle_info;
    }

    std::pair<Vector2f, Vector2f> _get_nearest_robot_full_info(const Vector2f& my_pos, float my_yaw, const Vector2f& my_vel, const OtherStatesMap& other_states) {
        if (other_states.empty()) return {Vector2f(10.0f, 10.0f), Vector2f::Zero()};
        float min_dist_sq = std::numeric_limits<float>::max();
        Vector2f nearest_pos, nearest_vel;
        for (const auto& pair : other_states) {
            float d_sq = (pair.second.first - my_pos).squaredNorm();
            if (d_sq < min_dist_sq) { min_dist_sq = d_sq; nearest_pos = pair.second.first; nearest_vel = pair.second.second; }
        }
        Vector2f relative_pos_world = nearest_pos - my_pos;
        Matrix2f rotation_matrix;
        rotation_matrix << std::cos(-my_yaw), -std::sin(-my_yaw), std::sin(-my_yaw), std::cos(-my_yaw);
        Vector2f relative_pos_robot = rotation_matrix * relative_pos_world;
        relative_pos_robot.x() = std::min(10.0f, std::max(-10.0f, relative_pos_robot.x()));
        relative_pos_robot.y() = std::min(10.0f, std::max(-10.0f, relative_pos_robot.y()));
        float linear_v = my_vel.x();
        Vector2f my_vel_vec(linear_v * std::cos(my_yaw), linear_v * std::sin(my_yaw));
        Vector2f relative_vel_world = nearest_vel - my_vel_vec;
        Vector2f relative_vel_robot_frame = rotation_matrix * relative_vel_world;
        return {relative_pos_robot, relative_vel_robot_frame};
    }
    
    float _calculate_remaining_path_distance(const PointsVector& path, int current_progress_idx, float total_path_len) {
        if (path.empty() || current_progress_idx >= path.size() - 1 || total_path_len < 1e-6f) return 0.0f;
        float remaining_dist = 0.0f;
        for (size_t i = current_progress_idx; i < path.size() - 1; ++i) {
            remaining_dist += (path[i+1] - path[i]).norm();
        }
        return remaining_dist;
    }
    
    void _publish_future_points_marker(const PointsVector& points_in_world_frame, const std::string& robot_name) {
        if (points_in_world_frame.empty()) return;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "future_points_" + robot_name;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1; marker.scale.y = 0.1;
        marker.color.g = 1.0f; marker.color.a = 1.0f;
        marker.lifetime = ros::Duration(0.5);
        for (const auto& point_xy : points_in_world_frame) {
            geometry_msgs::Point p;
            p.x = point_xy.x(); p.y = point_xy.y(); p.z = 0.2;
            marker.points.push_back(p);
        }
        marker_pub_.publish(marker);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "observation_calculator_node_cpp");
    ObservationCalculatorNode node;
    node.run();
    return 0;
}