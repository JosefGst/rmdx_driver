#ifndef MYACTUATOR_RMDX
#define MYACTUATOR_RMDX

#include <string>
#include <iostream>
#include <cstdio>
#include <signal.h>
#include <cmath>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"
#include "crc_check.h"

#include <chrono>

using namespace std;

class RMDX
{
private:
    chrono::time_point<chrono::steady_clock> start, end;
    uint8_t hex_cmd[13] = {0};
    uint8_t receive_hex[13] = {0};

    const uint8_t HEADER = 0x3e;
    uint8_t ID = 0x00;
    const uint8_t DATA_LENGHT = 0x08;
    const uint8_t VEL_CMD = 0xa2;
    const uint8_t READ_STAT = 0x9c;
    const uint8_t RELEASE_BRAKE = 0x77;

    int32_t prev_vel_cmd = 0;
    int8_t vel_update_count = 0;

public:
    // motor config
    uint16_t SPEED_DIF_MIN = 50; // the diff between a new speed command must be bigger as this value, otherwise will not take new command
    uint16_t SPEED_DIF_MAX = 55;
    int16_t SPEED_MIN = 50;
    int32_t SPEED_MAX = 1000;
    uint8_t ERROR_FLAG = 0; // 1 = crc error, 2 = no message from motor

    void sleep(unsigned long milliseconds);

    serial::Serial _serial;
    void beginn(string port, int baudrate, uint8_t _ID = 0x00);

    // send vel command in degree pser sec
    uint8_t vel_cmd(int32_t speed_dps);
    // send reading status to motor, get the values with the getter functions, e.g. double get_temp()
    uint8_t read_stat();
    uint8_t release_brake();
    // returns value in between max and 0. if value is below min it will set to 0, lager as max will set to max
    int16_t limit(int32_t value, int32_t min, int16_t max);
    // too small changes will be ignored and too large ones trimmed
    int16_t limit_dif(int32_t value, int32_t prev_value, int32_t min, int16_t max);
    double get_temp();
    double get_torque();
    double get_speed_dps();
    double get_angle();
};

#endif