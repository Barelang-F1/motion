#include "ros/ros.h"            // Untuk ROS
#include "std_msgs/UInt16.h"    // Untuk pesan jarak

// Callback function untuk menerima pesan jarak
void distanceCallback(const std_msgs::UInt16::ConstPtr& msg) {
    // Cetak jarak yang diterima dari sensor
    ROS_INFO("Diterima Jarak: %d mm", msg->data);
}

int main(int argc, char **argv) {
    // Inisialisasi ROS node
    ros::init(argc, argv, "vl53l0x_subscriber");
    ros::NodeHandle nh;

    // Membuat subscriber untuk topik "vl53l0x_distance"
    ros::Subscriber distance_sub = nh.subscribe("vl53l0x_distance", 10, distanceCallback);

    // Loop untuk menunggu pesan yang masuk
    ros::spin();

    return 0;
}
