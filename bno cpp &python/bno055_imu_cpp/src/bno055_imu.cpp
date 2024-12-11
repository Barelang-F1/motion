#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>

#define BNO055_ADDRESS 0x28 // Alamat I2C BNO055
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_EULER_H_ADDR 0x1A
#define BNO055_EULER_R_ADDR 0x1C
#define BNO055_EULER_P_ADDR 0x1E

void setMode(int fd, uint8_t mode) {
    uint8_t buffer[2] = {BNO055_OPR_MODE_ADDR, mode};
    if (write(fd, buffer, 2) != 2) {
        ROS_ERROR("Failed to set mode.");
    }
    usleep(100000); // Tunggu mode diatur
}

void readEulerAngles(int fd, float &yaw, float &roll, float &pitch) {
    uint8_t reg[1] = {BNO055_EULER_H_ADDR};
    if (write(fd, reg, 1) != 1) {
        ROS_ERROR("Failed to write to I2C.");
        return;
    }

    uint8_t buffer[6];
    if (read(fd, buffer, 6) != 6) {
        ROS_ERROR("Failed to read from I2C.");
        return;
    }

    yaw = (buffer[0] | (buffer[1] << 8)) / 16.0;
    roll = (buffer[2] | (buffer[3] << 8)) / 16.0;
    pitch = (buffer[4] | (buffer[5] << 8)) / 16.0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bno055_imu_node");
    ros::NodeHandle nh;

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
    ros::Publisher euler_pub = nh.advertise<std_msgs::Float32MultiArray>("imu/euler", 10);
    ros::Publisher yaw_pub = nh.advertise<std_msgs::Float32>("imu/yaw", 10);
    ros::Publisher roll_pub = nh.advertise<std_msgs::Float32>("imu/roll", 10);
    ros::Publisher pitch_pub = nh.advertise<std_msgs::Float32>("imu/pitch", 10);

    // Membuka file I2C
    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        ROS_ERROR("Failed to open I2C bus.");
        return -1;
    }

    // Set I2C address
    if (ioctl(fd, I2C_SLAVE, BNO055_ADDRESS) < 0) {
        ROS_ERROR("Failed to set I2C address.");
        close(fd);
        return -1;
    }

    setMode(fd, 0x08); // Set to IMU mode

    ros::Rate rate(5); // 5 Hz

    while (ros::ok()) {
        float yaw, roll, pitch;
        readEulerAngles(fd, yaw, roll, pitch);

        // Buat pesan IMU
        sensor_msgs::Imu imu_msg;
        imu_msg.orientation.x = yaw;
        imu_msg.orientation.y = roll;
        imu_msg.orientation.z = pitch;

        // Buat pesan untuk sudut Euler
        std_msgs::Float32MultiArray euler_array;
        euler_array.data = {yaw, roll, pitch};

        // Buat pesan untuk sudut individu
        std_msgs::Float32 yaw_msg, roll_msg, pitch_msg;
        yaw_msg.data = yaw;
        roll_msg.data = roll;
        pitch_msg.data = pitch;

        // Publikasikan data
        imu_pub.publish(imu_msg);
        euler_pub.publish(euler_array);
        yaw_pub.publish(yaw_msg);
        roll_pub.publish(roll_msg);
        pitch_pub.publish(pitch_msg);

        rate.sleep();
    }

    close(fd);
    return 0;
}
