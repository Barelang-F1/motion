#include "../include/inverse.cpp"
#include "ros/ros.h"
#include <JetsonGPIO.h>
#include "std_msgs/String.h"
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include "std_msgs/UInt16.h"           // Untuk pesan jarak
#include <std_msgs/UInt16MultiArray.h> // Menggunakan UInt16MultiArray
#include <math.h>
#define BUT_PIN 6 // Pin 37 adalah GPIO26
bool but = false; // Variabel untuk status tombol
int step = 0;
// Variabel global untuk menyimpan perintah aktif dan data IMU
std::string current_command = "";
float imu_yaw = 0.0, imu_roll = 0.0, imu_pitch = 0.0;
// int kanan = 0;
// int kiri = 0;

int mulai()
{
    int dxl_comm_result = COMM_TX_FAIL; // Communication result

    uint8_t dxl_error = 0; // Dynamixel error
    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        return 0;
    }

    // IC();
    for (int i = 0; i < 18; i++)
    {
        // Enable Dynamixel#1 Torque
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_idku[i], ADDR_AX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Dynamixel#%d has been successfully connected \n", dxl_idku[i]);
        }
    }

    siap();

    // while (1)
    // {
    //     // maju_robot();
    // }

    // portHandler->closePort();
}

void maju()
{
    ROS_INFO("Robot bergerak maju");
    maju_robot(); // Tambahkan logika untuk perintah maju robot di sini
}

void berhenti_robot()
{
    ROS_INFO("Robot berhenti untuk mempertahankan heading.");
    siap();
    // Tambahkan logika untuk menghentikan semua motor robot
}

// Fungsi untuk memutar robot ke kanan
void kanan_robot1()
{
    ROS_INFO("Robot berputar ke kanan untuk mengoreksi heading.");
    kanan_robot();
    // Tambahkan logika untuk memutar robot ke kanan
}

void putar_kanan_robot1()
{
    ROS_INFO("Robot berputar ke kanan untuk mengoreksi heading.");
    putar_kanan();
    // Tambahkan logika untuk memutar robot ke kanan
}

// Fungsi untuk memutar robot ke kiri
void kiri_robot1()
{
    ROS_INFO("Robot berputar ke kiri untuk mengoreksi heading.");
    kiri_robot();
    // Tambahkan logika untuk memutar robot ke kiri
}
// Inisialisasi GPIO untuk tombol
void initGPIO()
{
    GPIO::setmode(GPIO::BCM);       // Gunakan nomor pin BOARD
    GPIO::setup(BUT_PIN, GPIO::IN); // Konfigurasi pin 26 sebagai input
}

void updateButtonStatus()
{
    if (GPIO::input(BUT_PIN) == GPIO::LOW) // LOW artinya tombol ditekan
    {
        but = true;
    }
    else
    {
        but = false;
    }
}
// // Callback function to process TOF sensor data
// void distanceCallback(const std_msgs::UInt16MultiArray::ConstPtr &msg)
// {
//     if (msg->data.size() >= 3)
//     {
//         int kanan = msg->data[0];
//         int kiri = msg->data[1];
//         int depan = msg->data[2];
//               // Menampilkan informasi sensor di log ROS
//         ROS_INFO("Sensor data received - Kanan: %d, Kiri: %d, Depan: %d", kanan, kiri, depan);
//     }
//     else
//     {
//         ROS_WARN("Sensor data incomplete. Size: %zu", msg->data.size());
//     }

// }

// // Logika pergerakan robot
// if (kanan > 150)
// {
//     kiri_robot();
//     ROS_INFO("Robot bergerak ke kiri (sensor 1 < 150)");
// }
// else if (kiri < 60)
// {
//     maju_robot();
//     ROS_INFO("Robot bergerak maju (sensor 1 < 20)");
// }
// else if (depan > 180)
// {
//     putar_kanan();
//     ROS_INFO("Robot bergerak maju (sensor 1 < 20)");
// }
// else
// {
//     berhenti_robot();
//     ROS_INFO("Yaw tidak dalam rentang, robot berhenti.");
// }

void imuCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    if (msg->data.size() >= 3)
    {
        imu_yaw = msg->data[0];
        imu_roll = msg->data[1];
        imu_pitch = msg->data[2];
    }
}

void distanceCallback(const std_msgs::UInt16MultiArray::ConstPtr &msg)
{
    if (msg->data.size() >= 3)
    {
        kanan = msg->data[0];
        kiri = msg->data[1];
        depan = msg->data[2];
    }
}
void prosesLogika()
{
    switch (step)
    {
    case 0:
        if (but)
        {
            step = 1;
            ROS_INFO("Tombol ditekan, robot mulai putar kanan.");
        }
        break;
    case 1:
        putar_kanan();
        if (imu_yaw >= 260 && imu_yaw <= 280) // Validasi data IMU
        {
            step = 2;
            ROS_INFO("Yaw dalam rentang 260-280, robot bergerak maju.");
        }
        break;
    case 2:
        maju_robot();
        if (kanan > 50) // Validasi data TOF kanan
        {
            step = 3;
            ROS_INFO("Sensor kanan > 50, robot bergerak ke kanan.");
        }
        break;
    case 3:
        kanan_robot();
        if (depan > 100) // Validasi data TOF depan
        {
            step = 4;
            ROS_INFO("Sensor depan > 100, robot bergerak ke kiri.");
        }
        break;
    case 4:
        kiri_robot();
        ROS_INFO("Robot bergerak ke kiri.");
        break;
    default:
        berhenti_robot();
        ROS_INFO("Robot berhenti.");
        break;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    // Inisialisasi GPIO untuk tombol
    initGPIO();
    if (!mulai())
    {
        return 1; // Gagal inisialisasi, keluar program
    }

    // Subscribing ke topik chatter
    // ros::Subscriber sub = n.subscribe("chatter", 1000, commandCallback);
    // Subscriber untuk menerima data IMU
    ros::Subscriber euler_sub = n.subscribe("imu/euler", 10, imuCallback);
    ros::Subscriber distance_sub = n.subscribe("sensor_distance", 10, distanceCallback);
    // Loop utama untuk menjalankan perintah aktif
    ros::Rate loop_rate(33); // Menentukan frekuensi loop (10 Hz)
    while (ros::ok())
    {
        updateButtonStatus(); // Perbarui status tombol
        prosesLogika();       // Jalankan logika utama
        // Memproses callback ROS
        ros::spinOnce();

        loop_rate.sleep();
    }
    GPIO::cleanup();
    return 0;
}
