#include <iostream>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"



class VehicleSimulator : public rclcpp::Node
{
    public:
        VehicleSimulator();
        ~VehicleSimulator();

    private:
        void cmd_vel_cb_( const geometry_msgs::msg::TwistStamped::SharedPtr msg );
        void timer_callback_();

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Clock clock_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        nav_msgs::msg::Odometry odom_;
        geometry_msgs::msg::TwistStamped current_vel_;
        double prev_yaw_rad_;
        double pos_x_m_, pos_y_m_;
        double yaw_rad_;
        rclcpp::Time prev_time_;
        bool is_init_;
};
