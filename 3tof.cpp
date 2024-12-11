#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <chrono>
#include <thread>
#include <vector>

#define VL53L0X_I2C_ADDR 0x29 // Default I2C address for VL53L0X
#define NEW_ADDR1 0x30        // New address for sensor 1
#define NEW_ADDR2 0x31        // New address for sensor 2
#define NEW_ADDR3 0x32        // New address for sensor 3
#define NEW_ADDR4 0x33        // New address for sensor 4

// Deklarasi fungsi sebelum definisi
void write_register(int file, uint8_t reg, uint8_t value); // Deklarasi untuk menghindari error

// Fungsi untuk menginisialisasi I2C untuk setiap sensor
int i2c_init(const char* i2c_device, uint8_t address) {
    int file = open(i2c_device, O_RDWR);
    if (file < 0) {
        std::cerr << "Gagal membuka I2C device!" << std::endl;
        return -1;
    }
    if (ioctl(file, I2C_SLAVE, address) < 0) {
        std::cerr << "Gagal mengakses sensor VL53L0X!" << std::endl;
        close(file);
        return -1;
    }
    return file;
}

// Fungsi untuk mengubah alamat I2C sensor
void set_sensor_address(int file, uint8_t new_addr) {
    write_register(file, 0x8A, new_addr); // Sekarang tidak akan error
}

// Fungsi untuk membaca register dari sensor
uint8_t read_register(int file, uint8_t reg) {
    uint8_t value;
    if (write(file, &reg, 1) != 1) {
        std::cerr << "Gagal menulis ke register!" << std::endl;
        return -1;
    }
    if (read(file, &value, 1) != 1) {
        std::cerr << "Gagal membaca register!" << std::endl;
        return -1;
    }
    return value;
}

// Fungsi untuk menulis ke register sensor
void write_register(int file, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    if (write(file, buffer, 2) != 2) {
        std::cerr << "Gagal menulis ke register!" << std::endl;
    }
}

// Fungsi untuk memulai pengukuran jarak
void start_measurement(int file) {
    write_register(file, 0x00, 0x01); // Mengatur register untuk memulai pengukuran
}

// Fungsi untuk membaca hasil pengukuran jarak
uint16_t read_distance(int file) {
    uint8_t high_byte = read_register(file, 0x1E); // Register untuk high byte
    uint8_t low_byte = read_register(file, 0x1F);  // Register untuk low byte
    return (high_byte << 8) | low_byte;
}

int main() {
    const char* i2c_device = "/dev/i2c-1";
    std::vector<uint8_t> sensor_addresses = {NEW_ADDR1, NEW_ADDR2, NEW_ADDR3, NEW_ADDR4};

    // Inisialisasi setiap sensor dengan alamat berbeda
    for (int i = 0; i < sensor_addresses.size(); ++i) {
        int file = i2c_init(i2c_device, VL53L0X_I2C_ADDR);
        if (file < 0) {
            return 1;
        }
        set_sensor_address(file, sensor_addresses[i]);
        close(file);
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Tunggu sebentar setelah set alamat
    }

    // Membaca jarak dari setiap sensor secara terus-menerus
    while (true) {
        for (uint8_t address : sensor_addresses) {
            int file = i2c_init(i2c_device, address);
            if (file < 0) {
                return 1;
            }

            start_measurement(file);
            std::this_thread::sleep_for(std::chrono::milliseconds(30)); // Tunggu sebentar untuk pengukuran

            uint16_t distance = read_distance(file);
            std::cout << "Jarak dari sensor pada alamat " << (int)address << ": " << distance << " mm" << std::endl;

            close(file);
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Kurangi delay antara pengukuran
        }
    }

    return 0;
}
