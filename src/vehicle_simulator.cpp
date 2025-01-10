#include "vehicle_simulator/vehicle_simulator.hpp"
#include <chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

VehicleSimulator::VehicleSimulator()
: Node("vehicle_simulator")
{
    // Initialize transform listener and broadcaster
    //tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    //auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    //    this->get_node_base_interface(),
    //    this->get_node_timers_interface());
    //tf_buffer_->setCreateTimerInterface(timer_interface);
    //tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel", 1, std::bind(&VehicleSimulator::cmd_vel_cb_, this, _1));
    odom_publisher_  = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
    timer_ = this->create_wall_timer( 10ms, std::bind(&VehicleSimulator::timer_callback_, this));

    current_vel_ = geometry_msgs::msg::TwistStamped();
    odom_ = nav_msgs::msg::Odometry();
    pos_x_m_ = 0.0;
    pos_y_m_ = 0.0;
    yaw_rad_ = 0.0;
    prev_yaw_rad_ = 0.0;
    is_init_ = false;
}

VehicleSimulator::~VehicleSimulator()
{
}

void VehicleSimulator::cmd_vel_cb_( const geometry_msgs::msg::TwistStamped::SharedPtr msg )
{
    current_vel_ = *msg;
}

void VehicleSimulator::timer_callback_()
{
    rclcpp::Clock ros_clock(RCL_ROS_TIME);

    if ( !is_init_ )
    {
        prev_time_ = ros_clock.now();
        is_init_ = true;
        return ;
    }

    rclcpp::Time current_time = ros_clock.now();

    // タイムアウト時は速度をゼロにする
    double vel_time = current_vel_.header.stamp.sec*10e9 + current_vel_.header.stamp.nanosec;
    //(ros_clock.now() - msg->header.stamp).nanoseconds();
    if ( current_time.nanoseconds() - vel_time > 0.05*1000000000.0 )
    {
        current_vel_.twist.linear.x = 0.0;
        current_vel_.twist.angular.z = 0.0;
    }

    double dt = (current_time.nanoseconds() - prev_time_.nanoseconds())/1000000000.0;
    double dx = dt*current_vel_.twist.linear.x*cos(yaw_rad_);
    double dy = dt*current_vel_.twist.linear.x*sin(yaw_rad_);
    double d_theta = dt*current_vel_.twist.angular.z;

    pos_x_m_ += dx;
    pos_y_m_ += dy;
    yaw_rad_ += d_theta;

    std::cout<<"vel: "<<current_vel_.twist.linear.x<<std::endl;
    std::cout<<"dt: "<<dt<<std::endl;
    std::cout<<"x: "<<pos_x_m_<<std::endl;
    std::cout<<"y: "<<pos_y_m_<<std::endl;
    std::cout<<"yaw: "<<yaw_rad_<<std::endl;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad_);
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

    // tf
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = pos_x_m_;
    odom_trans.transform.translation.y = pos_y_m_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    tf_broadcaster_->sendTransform(odom_trans);

    // nav_msgs
    nav_msgs::msg::Odometry odom_msg;
    odom_.header.stamp = current_time;
    odom_.header.frame_id = "odom";
    odom_.pose.pose.position.x = pos_x_m_;
    odom_.pose.pose.position.y = pos_y_m_;
    odom_.pose.pose.position.z = 0.0;
    odom_.pose.pose.orientation = odom_quat;
    odom_.child_frame_id = "base_link";
    odom_.twist.twist.linear.x = current_vel_.twist.linear.x;
    odom_.twist.twist.linear.y = 0.0;
    odom_.twist.twist.linear.z = 0.0;
    odom_.twist.twist.angular.x = 0.0;
    odom_.twist.twist.angular.y = 0.0;
    odom_.twist.twist.angular.z = current_vel_.twist.angular.z;
    odom_publisher_->publish(odom_);

    prev_time_ = current_time;
}