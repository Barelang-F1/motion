#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32  # Gunakan Float32 untuk sudut tunggal
from std_msgs.msg import Float32MultiArray  # Import untuk topik baru sudut Euler
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055

# Inisialisasi ROS node
rospy.init_node('bno055_imu_publisher')

# Publisher untuk topik IMU dan topik khusus untuk setiap sudut Euler
imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
euler_pub = rospy.Publisher('imu/euler', Float32MultiArray, queue_size=10)  # Topik baru untuk Euler angle
yaw_pub = rospy.Publisher('imu/yaw', Float32, queue_size=10)
roll_pub = rospy.Publisher('imu/roll', Float32, queue_size=10)
pitch_pub = rospy.Publisher('imu/pitch', Float32, queue_size=10)

# Inisialisasi I2C untuk sensor BNO055
i2c = I2C(1)  # Device is /dev/i2c-1
sensor = adafruit_bno055.BNO055_I2C(i2c)
sensor.mode = adafruit_bno055.IMUPLUS_MODE

# Fungsi untuk membuat pesan IMU dan mengirimkan sudut Euler
def create_imu_msg():
    imu_msg = Imu()

    # Baca data dari sensor BNO055
    euler = sensor.euler  # Baca sudut Euler (heading, roll, pitch)

    # Isi data IMU dengan sudut Euler (opsional)
    imu_msg.orientation.x = euler[0]  # Heading (yaw)
    imu_msg.orientation.y = euler[1]  # Roll
    imu_msg.orientation.z = euler[2]  # Pitch

    # Data linear acceleration dan angular velocity bisa diisi dengan sensor.acceleration dan sensor.gyro
    imu_msg.linear_acceleration.x = sensor.acceleration[0]
    imu_msg.linear_acceleration.y = sensor.acceleration[1]
    imu_msg.linear_acceleration.z = sensor.acceleration[2]

    imu_msg.angular_velocity.x = sensor.gyro[0]
    imu_msg.angular_velocity.y = sensor.gyro[1]
    imu_msg.angular_velocity.z = sensor.gyro[2]

    return imu_msg, euler

rate = rospy.Rate(5)  # 5 Hz

while not rospy.is_shutdown():
    imu_msg, euler_msg = create_imu_msg()

    # Buat pesan untuk publikasi sudut Euler
    euler_array = Float32MultiArray()
    euler_array.data = euler_msg

    # Publikasikan IMU dan setiap sudut Euler secara terpisah secara bersamaan
    imu_pub.publish(imu_msg)
    euler_pub.publish(euler_array)
    yaw_pub.publish(euler_msg[0])
    roll_pub.publish(euler_msg[1])
    pitch_pub.publish(euler_msg[2])

    rate.sleep()
