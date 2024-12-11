#include <iostream>
#include <fcntl.h>  // Untuk open()
#include <unistd.h> // Untuk read(), write(), close()
#include <sys/ioctl.h> // Untuk ioctl()
#include <linux/i2c-dev.h> // Untuk konstanta I2C_SLAVE
#include <chrono> // Untuk std::chrono
#include <thread> // Untuk std::this_thread::sleep_for

#define VL53L0X_I2C_ADDR 0x29  // Alamat I2C VL53L0X

// Fungsi untuk menginisialisasi I2C
int i2c_init(const char* i2c_device) {
    int file = open(i2c_device, O_RDWR);
    if (file < 0) {
        std::cerr << "Gagal membuka I2C device!" << std::endl;
        return -1;
    }
    if (ioctl(file, I2C_SLAVE, VL53L0X_I2C_ADDR) < 0) {
        std::cerr << "Gagal mengakses sensor VL53L0X!" << std::endl;
        close(file);
        return -1;
    }
    return file;
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

// Fungsi untuk menginisialisasi sensor VL53L0X
bool initSensor(int file) {
    // Menulis ke beberapa register untuk inisialisasi sensor
    write_register(file, 0x88, 0x00);  // Register 0x88
    write_register(file, 0x80, 0x01);  // Register 0x80
    write_register(file, 0xFF, 0x01);  // Register 0xFF
    write_register(file, 0x00, 0x00);  // Register 0x00
    write_register(file, 0x91, 0x3C);  // Register 0x91
    write_register(file, 0x00, 0x01);  // Register 0x00
    write_register(file, 0xFF, 0x00);  // Register 0xFF
    write_register(file, 0x80, 0x00);  // Register 0x80

    // Cek ID sensor (biasanya 0xEE)
    uint8_t id = read_register(file, 0xC0); // Mengambil ID sensor dari register
    if (id != 0xEE) {
        std::cerr << "Gagal mendeteksi sensor VL53L0X! ID sensor: 0x" << std::hex << (int)id << std::dec << std::endl;
        return false;
    }
    return true;
}

// Fungsi untuk memulai pengukuran jarak
void start_measurement(int file) {
    write_register(file, 0x00, 0x01);  // Menulis ke register untuk memulai pengukuran
}

// Fungsi untuk membaca hasil pengukuran jarak
uint16_t read_distance(int file) {
    uint8_t high_byte = read_register(file, 0x1E); // Register untuk high byte
    uint8_t low_byte = read_register(file, 0x1F);  // Register untuk low byte
    return (high_byte << 8) | low_byte;
}

int main() {
    // Inisialisasi I2C
    int file = i2c_init("/dev/i2c-1");
    if (file < 0) {
        return 1;
    }

    // Inisialisasi sensor VL53L0X
    if (!initSensor(file)) {
        close(file);
        return 1;
    }

    // Memulai pengukuran jarak
    start_measurement(file);

    // Membaca jarak secara terus-menerus
    while (true) {
        // Memulai pengukuran setiap kali dalam loop
        start_measurement(file);

        // Tunggu beberapa milidetik agar sensor selesai mengukur
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        uint16_t distance = read_distance(file);
        std::cout << "Jarak: " << distance << " mm" << std::endl;

        // Tunggu sebelum pengukuran berikutnya
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Menutup file I2C
    close(file);
    return 0;
}
