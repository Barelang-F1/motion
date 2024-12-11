#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

// Callback function for the IMU data
void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ROS_INFO("Received IMU Orientation: x: %f, y: %f, z: %f",
             msg->orientation.x, msg->orientation.y, msg->orientation.z);
}

// Callback function for Euler angles array
void eulerCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size() == 3) {
        ROS_INFO("Received Euler Angles: Yaw: %f, Roll: %f, Pitch: %f",
                 msg->data[0], msg->data[1], msg->data[2]);
    }
}

// Callback function for individual yaw, roll, and pitch
void yawCallback(const std_msgs::Float32::ConstPtr& msg) {
    ROS_INFO("Received Yaw: %f", msg->data);
}

void rollCallback(const std_msgs::Float32::ConstPtr& msg) {
    ROS_INFO("Received Roll: %f", msg->data);
}

void pitchCallback(const std_msgs::Float32::ConstPtr& msg) {
    ROS_INFO("Received Pitch: %f", msg->data);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_subscriber_node");
    ros::NodeHandle nh;

    // Subscribers for each topic
    ros::Subscriber imu_sub = nh.subscribe("imu/data", 10, imuDataCallback);
    ros::Subscriber euler_sub = nh.subscribe("imu/euler", 10, eulerCallback);
    ros::Subscriber yaw_sub = nh.subscribe("imu/yaw", 10, yawCallback);
    ros::Subscriber roll_sub = nh.subscribe("imu/roll", 10, rollCallback);
    ros::Subscriber pitch_sub = nh.subscribe("imu/pitch", 10, pitchCallback);

    ros::spin();
    return 0;
}
