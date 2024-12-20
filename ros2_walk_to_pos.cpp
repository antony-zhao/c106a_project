#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace UNITREE_LEGGED_SDK;

rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr high_sub;
rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr high_pub;
ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros;
float init_x, init_y;
bool initialized = false;
float goal_x, goal_y;
const float thresh = 0.1;
std::vector<std::pair<double, double>> waypoints;
int curr_idx = 0;


void readCSV(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "File not found" << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double firstValue, secondValue;
        
        if (ss >> firstValue) {
            if (ss.peek() == ',') ss.ignore();
            if (ss >> secondValue) {
                waypoints.push_back(std::make_pair(firstValue, secondValue));
            }
        }
    }
    file.close();
}

void highStateCallback(const ros2_unitree_legged_msgs::msg::HighState::SharedPtr msg)
{
    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    high_cmd_ros.level_flag = HIGHLEVEL;
    high_cmd_ros.mode = 0;
    high_cmd_ros.gait_type = 0;
    high_cmd_ros.speed_level = 0;
    high_cmd_ros.foot_raise_height = 0;
    high_cmd_ros.body_height = 0;
    high_cmd_ros.euler[0] = 0;
    high_cmd_ros.euler[1] = 0;
    high_cmd_ros.euler[2] = 0;
    high_cmd_ros.velocity[0] = 0.0f;
    high_cmd_ros.velocity[1] = 0.0f;
    high_cmd_ros.yaw_speed = 0.0f;
    high_cmd_ros.reserve = 0;
    if (!initialized) {
        init_x = msg->position[0];
        init_y = msg->position[1];
        goal_x = waypoints[curr_idx].first;
        goal_y = waypoints[curr_idx].second;
        curr_idx++;
        printf("x: %f, y: %f\n", goal_x, goal_y);
        initialized = true;
    }
    float pos_x = msg->position[0] - init_x;
    float pos_y = msg->position[1] - init_y;
    printf("x: %f, y: %f\n", goal_x, goal_y);
    printf("pos_x: %f, pos_y: %f\n", pos_x, pos_y);
    if (abs(pos_x - goal_x) > thresh || abs(pos_y - goal_y) > thresh) {
        if (abs(pos_x - goal_x) > thresh) {
            high_cmd_ros.mode = 2;
            high_cmd_ros.gait_type = 1;
            int sign_x = signbit(pos_x - goal_x) * 2 - 1;
            printf("sign: %d\n", sign_x);
            if (abs(pos_x - goal_x) < 1.2 * thresh) {
                high_cmd_ros.velocity[0] = sign_x * 0.1f; // -1  ~ +1
            }
            else {
                high_cmd_ros.velocity[0] = sign_x * 0.2f; // -1  ~ +1
            }
            high_cmd_ros.yaw_speed = 0;
            high_cmd_ros.foot_raise_height = 0.1;
        }
        if (abs(pos_y - goal_y) > thresh) {
            high_cmd_ros.mode = 2;
            high_cmd_ros.gait_type = 1;
            int sign_y = signbit(pos_y - goal_y) * 2 - 1;
            if (abs(pos_y - goal_y) < 1.2 * thresh) {
                high_cmd_ros.velocity[1] = sign_y * 0.1f; // -1  ~ +1
            }
            else {
                high_cmd_ros.velocity[1] = sign_y * 0.2f; // -1  ~ +1
            }
            high_cmd_ros.yaw_speed = 0;
            high_cmd_ros.foot_raise_height = 0.1;
        }
    }
    else {
        if (curr_idx < waypoints.size()) {
            goal_x = waypoints[curr_idx].first;
            goal_y = waypoints[curr_idx].second;
            curr_idx++;
        }
        high_cmd_ros.mode = 1;
    }
    high_pub->publish(high_cmd_ros);
}

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("node_ros2_walk_to_pos");
    // goal_x = 1.6;
    // goal_y = -0.8;

    readCSV("waypoints.csv");

    high_sub = node->create_subscription<ros2_unitree_legged_msgs::msg::HighState>("high_state", 1, highStateCallback);
    high_pub = node->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1);
    while (!initialized) {
        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        high_cmd_ros.level_flag = HIGHLEVEL;
        high_cmd_ros.mode = 0;
        high_cmd_ros.gait_type = 0;
        high_cmd_ros.speed_level = 0;
        high_cmd_ros.foot_raise_height = 0;
        high_cmd_ros.body_height = 0;
        high_cmd_ros.euler[0] = 0;
        high_cmd_ros.euler[1] = 0;
        high_cmd_ros.euler[2] = 0;
        high_cmd_ros.velocity[0] = 0.0f;
        high_cmd_ros.velocity[1] = 0.0f;
        high_cmd_ros.yaw_speed = 0.0f;
        high_cmd_ros.reserve = 0;
        high_cmd_ros.mode = 1;
        high_pub->publish(high_cmd_ros);
        rclcpp::spin_some(node);
    }

    rclcpp::spin(node);

    return 0;

}