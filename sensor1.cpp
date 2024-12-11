#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <iostream>

#define BNO055_ADDRESS 0x28 // Alamat I2C BNO055
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_EULER_H_ADDR 0x1A
#define BNO055_EULER_R_ADDR 0x1C
#define BNO055_EULER_P_ADDR 0x1E

void setMode(int fd, uint8_t mode) {
    uint8_t buffer[2] = {BNO055_OPR_MODE_ADDR, mode};
    if (write(fd, buffer, 2) != 2) {
        std::cerr << "Failed to set mode." << std::endl;
    }
    usleep(100000); // Tunggu mode diatur
}

void readEulerAngles(int fd, float &yaw, float &roll, float &pitch) {
    uint8_t reg[1] = {BNO055_EULER_H_ADDR};
    if (write(fd, reg, 1) != 1) {
        std::cerr << "Failed to write to I2C." << std::endl;
        return;
    }

    uint8_t buffer[6];
    if (read(fd, buffer, 6) != 6) {
        std::cerr << "Failed to read from I2C." << std::endl;
        return;
    }

    yaw = (buffer[0] | (buffer[1] << 8)) / 16.0;
    roll = (buffer[2] | (buffer[3] << 8)) / 16.0;
    pitch = (buffer[4] | (buffer[5] << 8)) / 16.0;
}

int main() {
    // Membuka file I2C
    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open I2C bus." << std::endl;
        return -1;
    }

    // Set I2C address
    if (ioctl(fd, I2C_SLAVE, BNO055_ADDRESS) < 0) {
        std::cerr << "Failed to set I2C address." << std::endl;
        close(fd);
        return -1;
    }

    setMode(fd, 0x08); // Set to IMU mode

    while (true) {
        float yaw, roll, pitch;
        readEulerAngles(fd, yaw, roll, pitch);

        // Tampilkan data Euler
        std::cout << "Yaw: " << yaw << ", Roll: " << roll << ", Pitch: " << pitch << std::endl;

        usleep(200000); // Tunggu 200 ms sebelum membaca lagi
    }

    close(fd);
    return 0;
}
