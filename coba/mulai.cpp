#include "inverse_header.h"
#include "math.h"
#include "icecream.hpp"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

void inverse_k(uint8_t dxl_id, float x, float y, float z, int kecepatan);
void polinomial_trj(uint32_t dxl_id, float xp1, float yp1, float zp1, float xp2, float yp2, float zp2, float xp3, float yp3, float zp3, float xp4, float yp4, float zp4);
void trj_lurus(uint32_t dxl_id, float x0, float y0, float z0, float x1, float y1, float z1);
void trj_langkah(uint32_t dxl_id, float x0, float y0, float z0, float x1, float y1, float zp);
void trj_start(uint32_t id_kakiL1, uint32_t id_kakiL2, uint32_t id_kakiL3, uint32_t id_kakiR1, uint32_t id_kakiR2, uint32_t id_kakiR3, int kecepatan, int dly_trj);

int main()
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
    // usleep(500000);
    // siapv2();
    while (1)
    {
        // maju_robot();
        //coba_maju();
        // kanan_robot();
        // mundur_robot();
         //coba_putar_kananv2();
        // coba_putar_kiriv2();
        // coba_mundur();
        // coba_majuv2();
    }

    // portHandler->closePort();
    return 0;
}

void inverse_k(uint8_t dxl_id, float x, float y, float z, int kecepatan)
{
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION);
    //  a. HITUNG THETA Coxa
    //  hitung theta c
    theta_c = radToServomx(atan2(y, x));
    x0 = sqrt((y * y) + (x * x));
    // b. HITUNG THETA Femur
    // hitung theta f1
    x1 = x0 - c; // pengurangan panjang x0 dan coxa
    theta_f1 = radToServoax(atan2(z, x1));
    // hitung panjang a
    a = sqrt((z * z) + (x1 * x1));
    // hitung f2
    theta_f2 = radToServoax(acos((pow(f, 2) + pow(a, 2) - pow(t, 2)) / (2 * a * f)));
    // hitung f
    theta_f = theta_f1 + theta_f2;
    // c. HITUNG THETA Tibia
    // hitung theta t
    theta_t = radToServoax(acos((pow(f, 2) + pow(t, 2) - pow(a, 2)) / (2 * f * t)) - 1.57);

    // d. Normalisasi 0 derajat servo
    if (dxl_id == R1 || dxl_id == R2 || dxl_id == R3)
    {
        theta_c_real = 2048 - theta_c;
        // IC(theta_c_real);IC(theta_c);
        theta_f_real = 512 - theta_f;
        theta_t_real = 512 + theta_t;
    }

    if (dxl_id == L1 || dxl_id == L2 || dxl_id == L3)
    {
        // theta_c_real = 4504 - theta_c;
        theta_c_real = 2048 - theta_c;
        // IC(theta_c_real);
        // IC(theta_c);
        theta_f_real = 512 - theta_f;
        theta_t_real = 512 + theta_t;
    }

    switch (dxl_id)
    {
    case R1:

        posisi_servo[0] = theta_t_real;
        posisi_servo[1] = theta_f_real;
        posisi_servo[2] = theta_c_real;

        R1_x = x;
        R1_y = y;
        R1_z = z;
        break;
    case R2:

        posisi_servo[3] = theta_t_real;
        posisi_servo[4] = theta_f_real;
        posisi_servo[5] = theta_c_real;

        R2_x = x;
        R2_y = y;
        R2_z = z;
        break;
    case R3:

        posisi_servo[6] = theta_t_real;
        posisi_servo[7] = theta_f_real;
        posisi_servo[8] = theta_c_real;

        R3_x = x;
        R3_y = y;
        R3_z = z;
        break;
    case L1:

        posisi_servo[9] = theta_t_real;
        posisi_servo[10] = theta_f_real;
        posisi_servo[11] = theta_c_real;

        L1_x = x;
        L1_y = y;
        L1_z = z;
        break;
    case L2:

        posisi_servo[12] = theta_t_real;
        posisi_servo[13] = theta_f_real;
        posisi_servo[14] = theta_c_real;

        L2_x = x;
        L2_y = y;
        L2_z = z;
        break;
    case L3:

        posisi_servo[15] = theta_t_real;
        posisi_servo[16] = theta_f_real;
        posisi_servo[17] = theta_c_real;

        L3_x = x;
        L3_y = y;
        L3_z = z;
        break;
    }

    for (int i = 0; i < 18; i++)
    {

        // Allocate goal position value into byte array
        param_goal_position[0] = DXL_LOBYTE(posisi_servo[i]);
        param_goal_position[1] = DXL_HIBYTE(posisi_servo[i]);

        // Add Dynamixel#1 goal position value to the Syncwrite storage
        dxl_addparam_result = groupSyncWrite.addParam(dxl_idku[i], param_goal_position);
        if (dxl_addparam_result != true)
        {
            printf("[ID:%03d] groupSyncWrite addparam failed", dxl_idku[i]);
            IC(i);
        }

        groupSyncWrite.txPacket();
        // if (dxl_comm_result != COMM_SUCCESS)
        //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }

    groupSyncWrite.clearParam();
}

void polinomial_trj(uint32_t dxl_id, float xp1, float yp1, float zp1, float xp2, float yp2, float zp2, float xp3, float yp3, float zp3, float xp4, float yp4, float zp4)
{
    float A, B, C, D; // utk perhitungan polinomial
    float px, py, pz; // hasil polinomial
    int nmr_data = 0; // no data array

    // hitung end point dengan polinomial

    for (float t = 0.0; t <= 1.009; t = t + iterasi)
    {
        // hitung polinomial
        A = pow((1 - t), 3);
        B = 3 * t * pow(1 - t, 2);
        C = 3 * pow(t, 2) * (1 - t);
        D = pow(t, 3);
        px = A * xp1 + B * xp2 + C * xp3 + D * xp4;
        py = A * yp1 + B * yp2 + C * yp3 + D * yp4;
        pz = A * zp1 + B * zp2 + C * zp3 + D * zp4;

        // simpan hasil perhitungan
        switch (dxl_id)
        {

        case R1:
            array_px_R1[nmr_data] = px;
            array_py_R1[nmr_data] = py;
            array_pz_R1[nmr_data] = pz;
            break;
        case R2:
            array_px_R2[nmr_data] = px;
            array_py_R2[nmr_data] = py;
            array_pz_R2[nmr_data] = pz;
            break;
        case R3:
            array_px_R3[nmr_data] = px;
            array_py_R3[nmr_data] = py;
            array_pz_R3[nmr_data] = pz;
            break;
        case L1:
            array_px_L1[nmr_data] = px;
            array_py_L1[nmr_data] = py;
            array_pz_L1[nmr_data] = pz;
            break;
        case L2:
            array_px_L2[nmr_data] = px;
            array_py_L2[nmr_data] = py;
            array_pz_L2[nmr_data] = pz;
            break;
        case L3:
            array_px_L3[nmr_data] = px;
            array_py_L3[nmr_data] = py;
            array_pz_L3[nmr_data] = pz;
            break;
        }

        nmr_data++;
    }

    jlh_data = nmr_data;
}

void trj_lurus(uint32_t dxl_id, float x0, float y0, float z0, float x1, float y1, float z1)
{
    float xp1, yp1, zp1; // titik vektor1
    float xp2, yp2, zp2; // titik vektor2
    float xp3, yp3, zp3; // titik vektor3
    float xp4, yp4, zp4; // titik vektor4

    // tentukan titik vektor polinomial
    xp1 = x0;
    yp1 = y0;
    zp1 = z0; // P1
    xp2 = x0;
    yp2 = y0;
    zp2 = z0; // P2
    xp3 = x1;
    yp3 = y1;
    zp3 = z1; // P3
    xp4 = x1;
    yp4 = y1;
    zp4 = z1; // P4
    polinomial_trj(dxl_id, xp1, yp1, zp1, xp2, yp2, zp2, xp3, yp3, zp3, xp4, yp4, zp4);
}

// trayektori langkah
void trj_langkah(uint32_t dxl_id, float x0, float y0, float z0, float x1, float y1, float zp)
{
    float xp1, yp1, zp1; // titik vektor1
    float xp2, yp2, zp2; // titik vektor2
    float xp3, yp3, zp3; // titik vektor3
    float xp4, yp4, zp4; // titik vektor4

    float z1;
    z1 = (zp - (0.25 * z0)) / 0.75;
    // tentukan titik vektor polinomial
    xp1 = x0;
    yp1 = y0;
    zp1 = z0; // P1
    xp2 = x0;
    yp2 = y0;
    zp2 = z1; // P2
    xp3 = x1;
    yp3 = y1;
    zp3 = z1; // P3
    xp4 = x1;
    yp4 = y1;
    zp4 = z0; // P4
    polinomial_trj(dxl_id, xp1, yp1, zp1, xp2, yp2, zp2, xp3, yp3, zp3, xp4, yp4, zp4);
}

// eksekusi trayektori
void trj_start(uint32_t id_kakiR1, uint32_t id_kakiR2, uint32_t id_kakiR3, uint32_t id_kakiL1, uint32_t id_kakiL2, uint32_t id_kakiL3, int kecepatan, int dly_trj)
{
    // Hitung hasil perhitungan tsb menggunakan IK
    for (int i = 0; i < jlh_data; i++)
    {

        if (id_kakiR1 == R1)
        {
            inverse_k(id_kakiR1, array_px_R1[i], array_py_R1[i], array_pz_R1[i], kecepatan);
        }

        if (id_kakiR2 == R2)
        {
            inverse_k(id_kakiR2, array_px_R2[i], array_py_R2[i], array_pz_R2[i], kecepatan);
        }

        if (id_kakiR3 == R3)
        {
            inverse_k(id_kakiR3, array_px_R3[i], array_py_R3[i], array_pz_R3[i], kecepatan);
        }

        if (id_kakiL1 == L1)
        {
            inverse_k(id_kakiL1, array_px_L1[i], array_py_L1[i], array_pz_L1[i], kecepatan);
        }

        if (id_kakiL2 == L2)
        {
            inverse_k(id_kakiL2, array_px_L2[i], array_py_L2[i], array_pz_L2[i], kecepatan);
        }

        if (id_kakiL3 == L3)
        {
            inverse_k(id_kakiL3, array_px_L3[i], array_py_L3[i], array_pz_L3[i], kecepatan);
        }

        usleep(dly_trj);
    }
}

void siap()
{
    trj_lurus(R1, 80, 0, -60, 80, 0, -60);
    trj_lurus(R2, 80, 0, -60, 80, 0, -60);
    trj_lurus(R3, 80, 0, -60, 80, 0, -60);

    trj_lurus(L1, 80, 0, -60, 80, 0, -60);
    trj_lurus(L2, 80, 0, -60, 80, 0, -60);
    trj_lurus(L3, 80, 0, -60, 80, 0, -60);

    // trj_lurus(R1, 40, 0, -60, 40, 0, -60);
    // trj_lurus(R2, 40, 0, -60, 40, 0, -60);
    // trj_lurus(R3, 40, 0, -60, 40, 0, -60);

    // trj_lurus(L1, 40, 0, -60, 40, 0, -60);
    // trj_lurus(L2, 40, 0, -60, 40, 0, -60);
    // trj_lurus(L3, 40, 0, -60, 40, 0, -60);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 25000);
    // trj_start(xx, xx, R3, xx, xx, xx,  ultra_fast, 25000);
}

void coba_putar_kananv2()
{
    // kanan
    trj_langkah(L1, L1_x, L1_y, L1_z, 80, -25, -40);
    trj_langkah(L3, L3_x, L3_y, L3_z, 80, -25, -40);
    trj_langkah(R2, R2_x, R2_y, R2_z, 80, -25, -40);

    trj_lurus(R1, R1_x, R1_y, R1_z, 80, 0, -60); // 90
    trj_lurus(R3, R3_x, R3_y, R3_z, 80, 0, -60); // 90
    trj_lurus(L2, L2_x, L2_y, L2_z, 80, 0, -60);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 20000);

    trj_langkah(R1, R1_x, R1_y, R1_z, 80, -25, -40);
    trj_langkah(R3, R3_x, R3_y, R3_z, 80, -25, -40);
    trj_langkah(L2, L2_x, L2_y, L2_z, 80, -25, -40);

    trj_lurus(L1, L1_x, L1_y, L1_z, 80, 0, -60); // 95
    trj_lurus(L3, L3_x, L3_y, L3_z, 80, 0, -60); // 100
    trj_lurus(R2, R2_x, R2_y, R2_z, 80, 0, -60);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 20000);
}

void coba_putar_kiriv2(){
     // kanan
    trj_langkah(L1, L1_x, L1_y, L1_z, 80, 25, -40);
    trj_langkah(L3, L3_x, L3_y, L3_z, 80, 25, -40);
    trj_langkah(R2, R2_x, R2_y, R2_z, 80, 25, -40);

    trj_lurus(R1, R1_x, R1_y, R1_z, 80, 0, -60); // 90
    trj_lurus(R3, R3_x, R3_y, R3_z, 80, 0, -60); // 90
    trj_lurus(L2, L2_x, L2_y, L2_z, 80, 0, -60);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 20000);

    trj_langkah(R1, R1_x, R1_y, R1_z, 80, 25, -40);
    trj_langkah(R3, R3_x, R3_y, R3_z, 80, 25, -40);
    trj_langkah(L2, L2_x, L2_y, L2_z, 80, 25, -40);

    trj_lurus(L1, L1_x, L1_y, L1_z, 80, 0, -60); // 95
    trj_lurus(L3, L3_x, L3_y, L3_z, 80, 0, -60); // 100
    trj_lurus(R2, R2_x, R2_y, R2_z, 80, 0, -60);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 20000);
}
void siapv2()
{
    trj_lurus(R1, 80, -40, -60, 80, -40, -60);
    trj_lurus(R2, 80, 0, -60, 80, 0, -60);
    trj_lurus(R3, 80, 10, -60, 80, 10, -60);

    trj_lurus(L1, 80, 40, -60, 80, 40, -60);
    trj_lurus(L2, 80, 0, -60, 80, 0, -60);
    trj_lurus(L3, 80, -10, -60, 80, -10, -60);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 25000);
}

void coba_majuv2()
{
    // kanan
    trj_langkah(L1, L1_x, L1_y, L1_z, 80, 25, -40);
    trj_langkah(L3, L3_x, L3_y, L3_z, 80, -25, -40);
    trj_langkah(R2, R2_x, R2_y, R2_z, 80, 15, -40);

    trj_lurus(R1, R1_x, R1_y, R1_z, 80, -40, -60); // 90
    trj_lurus(R3, R3_x, R3_y, R3_z, 80, 10, -60); // 90
    trj_lurus(L2, L2_x, L2_y, L2_z, 80, 0, -60);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 20000);

    trj_langkah(R1, R1_x, R1_y, R1_z, 80, -25, -40);
    trj_langkah(R3, R3_x, R3_y, R3_z, 80, 25, -40);
    trj_langkah(L2, L2_x, L2_y, L2_z, 80, -15, -40);

    trj_lurus(L1, L1_x, L1_y, L1_z, 80, 40, -60); // 95
    trj_lurus(L3, L3_x, L3_y, L3_z, 80, -10, -60); // 100
    trj_lurus(R2, R2_x, R2_y, R2_z, 80, 0, -60);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 20000);
}

void coba_maju()
{
    // kanan
    trj_langkah(L1, L1_x, L1_y, L1_z, 80, -35, -40);
    trj_langkah(L3, L3_x, L3_y, L3_z, 80, 35, -40);
    trj_langkah(R2, R2_x, R2_y, R2_z, 120, 0, -40);

    trj_lurus(R1, R1_x, R1_y, R1_z, 80, 35, -60); // 90
    trj_lurus(R3, R3_x, R3_y, R3_z, 80, -35, -60); // 90
    trj_lurus(L2, L2_x, L2_y, L2_z, 120, 0, -60);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 30000);

    trj_langkah(R1, R1_x, R1_y, R1_z, 80, -35, -40);
    trj_langkah(R3, R3_x, R3_y, R3_z, 80, 35, -40);
    trj_langkah(L2, L2_x, L2_y, L2_z, 70, 0, -40);

    trj_lurus(L1, L1_x, L1_y, L1_z, 80, 0, -60); // 95
    trj_lurus(L3, L3_x, L3_y, L3_z, 80, 0, -60); // 100
    trj_lurus(R2, R2_x, R2_y, R2_z, 70, 0, -60);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 30000);
}
void coba_mundur()
{
    // kanan
    trj_langkah(L1, L1_x, L1_y, L1_z, 80, 15, -40);
    trj_langkah(L3, L3_x, L3_y, L3_z, 80, -15, -40);
    trj_langkah(R2, R2_x, R2_y, R2_z, 70, 0, -40);

    trj_lurus(R1, R1_x, R1_y, R1_z, 80, 0, -60); // 90
    trj_lurus(R3, R3_x, R3_y, R3_z, 80, 0, -60); // 90
    trj_lurus(L2, L2_x, L2_y, L2_z, 80, 0, -60);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 20000);

    trj_langkah(R1, R1_x, R1_y, R1_z, 80, 15, -40);
    trj_langkah(R3, R3_x, R3_y, R3_z, 80, -15, -40);
    trj_langkah(L2, L2_x, L2_y, L2_z, 90, 0, -40);

    trj_lurus(L1, L1_x, L1_y, L1_z, 80, 0, -60); // 95
    trj_lurus(L3, L3_x, L3_y, L3_z, 80, 0, -60); // 100
    trj_lurus(R2, R2_x, R2_y, R2_z, 80, 0, -60);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 20000);
}
void maju_robot()
{

    // kanan
    trj_langkah(L1, L1_x, L1_y, L1_z, 40, -10, -60);
    trj_langkah(L3, L3_x, L3_y, L3_z, 40, -10, -60);
    trj_langkah(R2, R2_x, R2_y, R2_z, 40, 5, -60);

    trj_lurus(R1, R1_x, R1_y, R1_z, 40, 0, -80); // 40
    trj_lurus(R3, R3_x, R3_y, R3_z, 40, 0, -80); // 40
    trj_lurus(L2, L2_x, L2_y, L2_z, 40, 0, -80);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 15000);

    trj_langkah(R1, R1_x, R1_y, R1_z, 40, 10, -60);
    trj_langkah(R3, R3_x, R3_y, R3_z, 40, 10, -60);
    trj_langkah(L2, L2_x, L2_y, L2_z, 40, -5, -60);

    trj_lurus(L1, L1_x, L1_y, L1_z, 40, 0, -80); // 95
    trj_lurus(L3, L3_x, L3_y, L3_z, 40, 0, -80); // 00
    trj_lurus(R2, R2_x, R2_y, R2_z, 40, 0, -80);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 15000);
}

void mundur_robot()
{
    // MUNDUR //belum buat
    trj_langkah(L1, L1_x, L1_y, L1_z, 40, 10, -60);
    trj_langkah(L3, L3_x, L3_y, L3_z, 40, 10, -60);
    trj_langkah(R2, R2_x, R2_y, R2_z, 40, -10, -60);

    trj_lurus(R1, R1_x, R1_y, R1_z, 40, 0, -95); // 90
    trj_lurus(R3, R3_x, R3_y, R3_z, 40, 0, -70); // 90
    trj_lurus(L2, L2_x, L2_y, L2_z, 40, 0, -90);

    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 15000);

    trj_langkah(R1, R1_x, R1_y, R1_z, 40, -10, -60);
    trj_langkah(R3, R3_x, R3_y, R3_z, 40, -10, -60);
    trj_langkah(L2, L2_x, L2_y, L2_z, 40, 10, -60);

    trj_lurus(L1, L1_x, L1_y, L1_z, 40, 0, -90); // 95
    trj_lurus(L3, L3_x, L3_y, L3_z, 40, 0, -90); // 100
    trj_lurus(R2, R2_x, R2_y, R2_z, 40, 0, -90);

    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 15000);
}

void kiri_robot()
{
    // kanan
    trj_langkah(L1, L1_x, L1_y, L1_z, 30, 0, -60);
    trj_langkah(L3, L3_x, L3_y, L3_z, 30, 0, -60);
    trj_langkah(R2, R2_x, R2_y, R2_z, 50, 0, -60);

    trj_lurus(R1, R1_x, R1_y, R1_z, 40, 0, -80); // 40
    trj_lurus(R3, R3_x, R3_y, R3_z, 40, 0, -80); // 40
    trj_lurus(L2, L2_x, L2_y, L2_z, 40, 0, -80);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 15000);

    trj_langkah(R1, R1_x, R1_y, R1_z, 50, 0, -60);
    trj_langkah(R3, R3_x, R3_y, R3_z, 50, 0, -60);
    trj_langkah(L2, L2_x, L2_y, L2_z, 30, 0, -60);

    trj_lurus(L1, L1_x, L1_y, L1_z, 40, 0, -80); // 95
    trj_lurus(L3, L3_x, L3_y, L3_z, 40, 0, -80); // 00
    trj_lurus(R2, R2_x, R2_y, R2_z, 40, 0, -80);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 15000);
}

void kanan_robot()
{
    // kanan
    trj_langkah(L1, L1_x, L1_y, L1_z, 30, 0, -60);
    trj_langkah(L3, L3_x, L3_y, L3_z, 30, 0, -60);
    trj_langkah(R2, R2_x, R2_y, R2_z, 50, 0, -60);

    trj_lurus(R1, R1_x, R1_y, R1_z, 40, 0, -80); // 40
    trj_lurus(R3, R3_x, R3_y, R3_z, 40, 0, -80); // 40
    trj_lurus(L2, L2_x, L2_y, L2_z, 40, 0, -80);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 15000);

    trj_langkah(R1, R1_x, R1_y, R1_z, 50, 0, -60);
    trj_langkah(R3, R3_x, R3_y, R3_z, 50, 0, -60);
    trj_langkah(L2, L2_x, L2_y, L2_z, 30, 0, -60);

    trj_lurus(L1, L1_x, L1_y, L1_z, 40, 0, -80); // 95
    trj_lurus(L3, L3_x, L3_y, L3_z, 40, 0, -80); // 00
    trj_lurus(R2, R2_x, R2_y, R2_z, 40, 0, -80);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 15000);
}

void coba_putar_kanan()
{

    // langkah kaki kiri
    trj_langkah(L1, L1_x, L1_y, L1_z, 40, -15, -75); // x0,y0,x1,y1,z0,zp
    trj_langkah(L3, L3_x, L3_y, L3_z, 40, -15, -70);
    trj_langkah(R2, R2_x, R2_y, R2_z, 40, -15, -70);
    // geser kaki kanan
    trj_lurus(R1, R1_x, R1_y, R1_z, 40, 0, -95);
    trj_lurus(R3, R3_x, R3_y, R3_z, 40, 0, -70);
    trj_lurus(L2, L2_x, L2_y, L2_z, 40, 0, -90);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 15000);
    // langkah kaki kanan
    trj_langkah(R1, R1_x, R1_y, R1_z, 40, -15, -70);
    trj_langkah(R3, R3_x, R3_y, R3_z, 40, -15, -50);
    trj_langkah(L2, L2_x, L2_y, L2_z, 40, -15, -70);
    // geser kaki kanan
    trj_lurus(L1, L1_x, L1_y, L1_z, 40, 0, -90);
    trj_lurus(L3, L3_x, L3_y, L3_z, 40, 0, -90);
    trj_lurus(R2, R2_x, R2_y, R2_z, 40, 0, -90);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 15000);
}
void putar_kanan()
{

    // langkah kaki kiri
    trj_langkah(L1, L1_x, L1_y, L1_z, -40, 15, -75); // x0,y0,x1,y1,z0,zp
    trj_langkah(L3, L3_x, L3_y, L3_z, -40, 15, -70);
    trj_langkah(R2, R2_x, R2_y, R2_z, 40, -15, -70);
    // geser kaki kanan
    trj_lurus(R1, R1_x, R1_y, R1_z, 40, 0, -95);
    trj_lurus(R3, R3_x, R3_y, R3_z, 40, 0, -70);
    trj_lurus(L2, L2_x, L2_y, L2_z, -40, 0, -90);
    trj_start(L1, L2, L3, R1, R2, R3, ultra_fast, 15000);
    // langkah kaki kanan
    trj_langkah(R1, R1_x, R1_y, R1_z, 40, -15, -70);
    trj_langkah(R3, R3_x, R3_y, R3_z, 40, -15, -50);
    trj_langkah(L2, L2_x, L2_y, L2_z, -40, 15, -70);
    // geser kaki kanan
    trj_lurus(L1, L1_x, L1_y, L1_z, -40, 0, -90);
    trj_lurus(L3, L3_x, L3_y, L3_z, -40, 0, -90);
    trj_lurus(R2, R2_x, R2_y, R2_z, 40, 0, -90);
    trj_start(L1, L2, L3, R1, R2, R3, ultra_fast, 15000);
}
void coba_putar_kiri()
{
    // langkah kaki kiri
    trj_langkah(L1, L1_x, L1_y, L1_z, 40, 15, -70); // x0,y0,x1,y1,z0,zp
    trj_langkah(L3, L3_x, L3_y, L3_z, 40, 15, -70);
    trj_langkah(R2, R2_x, R2_y, R2_z, 40, 15, -70);
    // geser kaki kanan
    trj_lurus(R1, R1_x, R1_y, R1_z, 40, 0, -95);
    trj_lurus(R3, R3_x, R3_y, R3_z, 40, 0, -70);
    trj_lurus(L2, L2_x, L2_y, L2_z, 40, 0, -90);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 15000);
    // langkah kaki kanan
    trj_langkah(R1, R1_x, R1_y, R1_z, 40, 15, -75);
    trj_langkah(R3, R3_x, R3_y, R3_z, 40, 15, -50);
    trj_langkah(L2, L2_x, L2_y, L2_z, 40, 15, -70);
    // geser kaki kanan
    trj_lurus(L1, L1_x, L1_y, L1_z, 40, 0, -90);
    trj_lurus(L3, L3_x, L3_y, L3_z, 40, 0, -90);
    trj_lurus(R2, R2_x, R2_y, R2_z, 40, 0, -90);
    trj_start(R1, R2, R3, L1, L2, L3, ultra_fast, 15000);
}
void putar_kiri()
{
    // langkah kaki kiri
    trj_langkah(L1, L1_x, L1_y, L1_z, -40, -15, -70); // x0,y0,x1,y1,z0,zp
    trj_langkah(L3, L3_x, L3_y, L3_z, -40, -15, -70);
    trj_langkah(R2, R2_x, R2_y, R2_z, 40, 15, -70);
    // geser kaki kanan
    trj_lurus(R1, R1_x, R1_y, R1_z, 40, 0, -95);
    trj_lurus(R3, R3_x, R3_y, R3_z, 40, 0, -70);
    trj_lurus(L2, L2_x, L2_y, L2_z, -40, 0, -90);
    trj_start(L1, L2, L3, R1, R2, R3, ultra_fast, 15000);
    // langkah kaki kanan
    trj_langkah(R1, R1_x, R1_y, R1_z, 40, 15, -75);
    trj_langkah(R3, R3_x, R3_y, R3_z, 40, 15, -50);
    trj_langkah(L2, L2_x, L2_y, L2_z, -40, -15, -70);
    // geser kaki kanan
    trj_lurus(L1, L1_x, L1_y, L1_z, -40, 0, -90);
    trj_lurus(L3, L3_x, L3_y, L3_z, -40, 0, -90);
    trj_lurus(R2, R2_x, R2_y, R2_z, 40, 0, -90);
    trj_start(L1, L2, L3, R1, R2, R3, ultra_fast, 15000);
}

void tanggav2()
{

    // langkah kaki kanan
    trj_langkah(R1, R1_x, R1_y, R1_z, 80, 0, -15);
    trj_langkah(R3, R3_x, R3_y, R3_z, 80, 0, -15);
    trj_langkah(L2, L2_x, L2_y, L2_z, -15, 0, -25); // 10
    // geser kaki kanan
    trj_lurus(L1, L1_x, L1_y, L1_z, -80, 0, -105);
    trj_lurus(L3, L3_x, L3_y, L3_z, -80, 0, -105);
    trj_lurus(R2, R2_x, R2_y, R2_z, 15, 0, -85);
    trj_start(L1, L2, L3, R1, R2, R3, slow, 15000);

    // langkah kaki kiri
    trj_langkah(L1, L1_x, L1_y, L1_z, -25, 0, -25); // x0,y0,x1,y1,z0,zp
    trj_langkah(L3, L3_x, L3_y, L3_z, -25, 0, -25);
    trj_langkah(R2, R2_x, R2_y, R2_z, 80, 0, -10); // 25
    // geser kaki kanan
    trj_lurus(R1, R1_x, R1_y, R1_z, 25, 0, -85);
    trj_lurus(R3, R3_x, R3_y, R3_z, 25, 0, -85);
    trj_lurus(L2, L2_x, L2_y, L2_z, -65, 0, -105);
    trj_start(L1, L2, L3, R1, R2, R3, slow, 15000);
}

void tangga_korban4()
{
    // langkah kaki kiri
    trj_langkah(L1, L1_x, L1_y, L1_z, -80, 0, -15); // x0,y0,x1,y1,z0,zp
    trj_langkah(L3, L3_x, L3_y, L3_z, -80, 0, -15);
    trj_langkah(R2, R2_x, R2_y, R2_z, 15, 0, -25); // 25
    // geser kaki kanan
    trj_lurus(R1, R1_x, R1_y, R1_z, 80, 0, -105);
    trj_lurus(R3, R3_x, R3_y, R3_z, 80, 0, -105);
    trj_lurus(L2, L2_x, L2_y, L2_z, -15, 0, -85);
    trj_start(L1, L2, L3, R1, R2, R3, med, 15);
    // langkah kaki kanan
    trj_langkah(R1, R1_x, R1_y, R1_z, 25, 0, -35);
    trj_langkah(R3, R3_x, R3_y, R3_z, 25, 0, -35);
    trj_langkah(L2, L2_x, L2_y, L2_z, -80, 0, -10); // 10
    // geser kaki kanan
    trj_lurus(L1, L1_x, L1_y, L1_z, -25, 0, -85);
    trj_lurus(L3, L3_x, L3_y, L3_z, -25, 0, -85);
    trj_lurus(R2, R2_x, R2_y, R2_z, 65, 0, -105);
    trj_start(L1, L2, L3, R1, R2, R3, med, 15);
}

void tanggav2_korban4()
{
    // langkah kaki kiri
    trj_langkah(L1, L1_x, L1_y, L1_z, -25, 0, -15); // x0,y0,x1,y1,z0,zp
    trj_langkah(L3, L3_x, L3_y, L3_z, -25, 0, -15);
    trj_langkah(R2, R2_x, R2_y, R2_z, 65, 0, -25); // 25
    // geser kaki kanan
    trj_lurus(R1, R1_x, R1_y, R1_z, 15, 0, -105);
    trj_lurus(R3, R3_x, R3_y, R3_z, 15, 0, -105);
    trj_lurus(L2, L2_x, L2_y, L2_z, -80, 0, -85);
    trj_start(L1, L2, L3, R1, R2, R3, med, 15);
    // langkah kaki kanan
    trj_langkah(R1, R1_x, R1_y, R1_z, 80, 0, -35);
    trj_langkah(R3, R3_x, R3_y, R3_z, 80, 0, -35);
    trj_langkah(L2, L2_x, L2_y, L2_z, -25, 0, -10); // 10
    // geser kaki kanan
    trj_lurus(L1, L1_x, L1_y, L1_z, -80, 0, -85);
    trj_lurus(L3, L3_x, L3_y, L3_z, -80, 0, -85);
    trj_lurus(R2, R2_x, R2_y, R2_z, 25, 0, -105);
    trj_start(L1, L2, L3, R1, R2, R3, med, 15);
}