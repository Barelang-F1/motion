#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <chrono>
#include <thread>
#include <signal.h>
#include <JetsonGPIO.h>
#include <iomanip>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#define VL53L0X_DEFAULT_ADDRESS 0x29
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x32

#define SHT_LOX1 19 // GPIO untuk sensor 1
#define SHT_LOX2 13 // GPIO untuk sensor 2

static volatile bool keep_running = true;
std::mutex i2c_mutex;

void signalHandler(int signal)
{
    keep_running = false;
    std::cout << "Signal " << signal << " received. Cleaning up GPIO..." << std::endl;
    GPIO::cleanup();
}

int i2c_init(const char *i2c_device, uint8_t address)
{
    int file = open(i2c_device, O_RDWR);
    if (file < 0)
    {
        std::cerr << "Gagal membuka I2C device: " << i2c_device << std::endl;
        return -1;
    }
    if (ioctl(file, I2C_SLAVE, address) < 0)
    {
        std::cerr << "Gagal mengakses sensor pada alamat: " << static_cast<int>(address) << std::endl;
        close(file);
        return -1;
    }
    return file;
}

void write_register(int file, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    if (write(file, buffer, 2) != 2)
    {
        std::cerr << "Gagal menulis " << static_cast<int>(value) << " ke register " << static_cast<int>(reg) << std::endl;
    }
}

uint8_t read_register(int file, uint8_t reg)
{
    uint8_t value;
    if (write(file, &reg, 1) != 1)
    {
        std::cerr << "Gagal menulis ke register!" << std::endl;
        return -1;
    }
    if (read(file, &value, 1) != 1)
    {
        std::cerr << "Gagal membaca register!" << std::endl;
        return -1;
    }
    return value;
}

uint16_t read_distance(int file)
{
    uint8_t high_byte = read_register(file, 0x1E);
    uint8_t low_byte = read_register(file, 0x1F);
    return (high_byte << 8) | low_byte;
}

void reset_sensor(int gpio_pin, uint8_t new_address)
{
    GPIO::output(gpio_pin, GPIO::LOW);
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Delay lebih lama
    GPIO::output(gpio_pin, GPIO::HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Delay lebih lama

    int file = i2c_init("/dev/i2c-1", VL53L0X_DEFAULT_ADDRESS);
    if (file >= 0)
    {
        write_register(file, 0x8A, new_address);
        close(file);
    }
}
uint16_t read_distance_retry(int file, int retries = 3)
{
    uint16_t distance = 0;
    while (retries-- > 0)
    {
        {
            std::lock_guard<std::mutex> lock(i2c_mutex);
            distance = read_distance(file);
        }
        if (distance != 0)
            break;                                                  // Berhasil membaca
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Tunggu sebelum mencoba lagi
    }
    return distance;
}

void setup_sensors()
{
    GPIO::setmode(GPIO::BCM);
    GPIO::setup(SHT_LOX1, GPIO::OUT, GPIO::LOW);
    GPIO::setup(SHT_LOX2, GPIO::OUT, GPIO::LOW);

    // Reset semua sensor
    GPIO::output(SHT_LOX1, GPIO::LOW);
    GPIO::output(SHT_LOX2, GPIO::LOW);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Unreset semua sensor
    GPIO::output(SHT_LOX1, GPIO::HIGH);
    GPIO::output(SHT_LOX2, GPIO::HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Atur sensor 1
    reset_sensor(SHT_LOX1, LOX1_ADDRESS);
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Tunggu stabilisasi

    // Atur sensor 2
    reset_sensor(SHT_LOX2, LOX2_ADDRESS);
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Tunggu stabilisasi
}

bool is_sensor_ready(int file)
{
    uint8_t ready_status = read_register(file, 0x13); // Contoh register status
    return (ready_status & 0x01);                     // Periksa bit siap
}

void measure_distance(int file1, int file2, ros::Publisher &distance_pub)
{
    uint16_t distance1 = 0;
    uint16_t distance2 = 0;
    if (is_sensor_ready(file1))
        distance1 = read_distance_retry(file1);
    if (is_sensor_ready(file2))
        distance2 = read_distance_retry(file2);

    {
        std::lock_guard<std::mutex> lock(i2c_mutex);
        write_register(file1, 0x00, 0x01); // Memulai pengukuran untuk sensor 1
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        distance1 = read_distance(file1);
    }

    {
        std::lock_guard<std::mutex> lock(i2c_mutex);
        write_register(file2, 0x00, 0x01); // Memulai pengukuran untuk sensor 2
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        distance2 = read_distance(file2);
    }

    // Publikasi hasil ke ROS
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Sensor 1: " << distance1 << " mm, Sensor 2: " << distance2 << " mm";
    msg.data = ss.str();
    distance_pub.publish(msg);

    // Log ke terminal
    std::cout << "Sensor 1: " << distance1 << " mm, Sensor 2: " << distance2 << " mm" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "distance_sensor_node");
    ros::NodeHandle nh;
    ros::Publisher distance_pub = nh.advertise<std_msgs::String>("sensor_distance", 10);

    signal(SIGINT, signalHandler); // Menambahkan signal handler
    setup_sensors();

    int file1 = i2c_init("/dev/i2c-1", LOX1_ADDRESS);
    int file2 = i2c_init("/dev/i2c-1", LOX2_ADDRESS);

    if (file1 < 0 || file2 < 0)
    {
        std::cerr << "Gagal membuka sensor VL53L0X!" << std::endl;
        return 1;
    }

    while (ros::ok() && keep_running)
    {
        measure_distance(file1, file2, distance_pub);
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    close(file1);
    close(file2);
    GPIO::cleanup();
    return 0;
}
