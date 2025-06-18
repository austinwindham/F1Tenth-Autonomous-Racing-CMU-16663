#include <sstream>
#include <string>
#include <fstream>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
/// CHECK: include needed ROS msg type headers and libraries

#include "rclcpp/logging.hpp"
#include "nav_msgs/msg/path.hpp"


using namespace std;

vector<vector<float>> raw_waypoints;

class PurePursuit : public rclcpp::Node
{

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        // TODO: create ROS subscribers and publishers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_sim_topic, 10, std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_visualizer", 10);

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
                "/planned_path", 10, std::bind(&PurePursuit::path_callback, this, std::placeholders::_1));
            
        
    }

    

private:
    string odom_sim_topic = "/ego_racecar/odom";
    string drive_topic = "/drive";
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    vector<vector<float>> current_path;  // replaces raw_waypoints


    float lookahead_distance = 0.7;

    vector<float> find_closest_waypoint(float x, float y, float theta) 
    {

        if (current_path.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No path received yet.");
            return vector<float>{0.0, 0.0};  // dummy point to avoid crashing
        }

        float min_dist = 1000000;
        int closest_idx = 0;
        for (int i = 0; i < current_path.size(); i++)
        {
            float dist = sqrt(pow(x - current_path[i][0], 2) + pow(y - current_path[i][1], 2));
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }

        vector<float> next_waypoint;
        bool found_pt = false;
        bool found_notvalid = false;
        float test = 0;
        float testangle = 0;
        float testtheta = 0;
        vector<float> testpoint;


        for (int j = 0; j < current_path.size(); j++) {
            float dist = sqrt(pow(x - current_path[j][0], 2) + pow(y - current_path[j][1], 2));
            if (abs(dist - lookahead_distance) < 0.2) {/// changed from .01
                float angle = atan2(current_path[j][1] - y, current_path[j][0] - x);
                found_notvalid = true;
                test = abs(angle - theta);
                testangle = angle;
                testtheta = theta;
                testpoint = current_path[j];
                if (cos(angle - theta) > 0){
                    next_waypoint = current_path[j];
                    found_pt = true;
                    break;
                }
                
            }
        }

        if (!found_pt)
        {
            RCLCPP_WARN(this->get_logger(), "No valid waypoint found, using current pose as dummy.");
            return vector<float>{x, y};  // fallback to avoid crash
        }

      
        return next_waypoint;
    }

    double get_heading(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    void waypoint_viz(vector<float> waypoint)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "waypoints";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = waypoint[0];
        marker.pose.position.y = waypoint[1];
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0; 
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_pub_->publish(marker);
    }
    
    void pose_callback(const nav_msgs::msg::Odometry::ConstPtr pose_msg)
    {

        if (current_path.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No path to follow yet.");
            return;
        }

        
        // TODO: find the current waypoint to track using methods mentioned in lecture
        float curr_x = pose_msg->pose.pose.position.x;
        float curr_y = pose_msg->pose.pose.position.y;
        float curr_theta = get_heading(pose_msg->pose.pose.orientation);
        vector<float> next_waypoint = find_closest_waypoint(curr_x, curr_y, curr_theta);

        waypoint_viz(next_waypoint);

        // TODO: transform goal point to vehicle frame of reference
        float x_rel = next_waypoint[0] - curr_x;
        float y_rel = next_waypoint[1] - curr_y;
        float x_car = cos(curr_theta) * x_rel + sin(curr_theta) *y_rel;
        float y_car = -sin(curr_theta)* x_rel + cos(curr_theta) * y_rel;


        // TODO: calculate curvature/steering angle
        float radius = (lookahead_distance * lookahead_distance) / (2 * y_car);
        float curvature = 1 / radius;
        float steering_angle = atan(lookahead_distance * curvature);


        // TODO: publish drive message, don't forget to limit the steering angle.
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = 0.4;
        drive_pub_->publish(drive_msg);
    }

    void path_callback(const nav_msgs::msg::Path::ConstSharedPtr path_msg)
    {
        current_path.clear();  // clear old path

        for (const auto& pose_stamped : path_msg->poses)
        {
            vector<float> waypoint;
            waypoint.push_back(pose_stamped.pose.position.x);
            waypoint.push_back(pose_stamped.pose.position.y);
            current_path.push_back(waypoint);
        }

        RCLCPP_INFO(this->get_logger(), "Received new path with %ld waypoints", current_path.size());
    }


    
};

int main(int argc, char **argv)
{

    // ifstream file("/sim_ws/src/pure_pursuit/waypoints.csv");

    // if (!file.is_open()) {
    //     cout << "Check file path" << endl;
    //     return 0;
    // }

    // string line;
    // while (getline(file, line)) {
    //     stringstream ss(line);
    //     string data;
    //     vector<float> row;
    //     while (getline(ss, data, ',')) {
    //         row.push_back(stof(data));
    //     }
    //     raw_waypoints.push_back(row);
    // }

    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}