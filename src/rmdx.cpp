#include "rmdx_driver/rmdx.h"

void RMDX::sleep(unsigned long milliseconds)
{
#ifdef _WIN32
    Sleep(milliseconds); // 100 ms
#else
    usleep(milliseconds * 1000); // 100 ms
#endif
}

void RMDX::beginn(string port, int baudrate, uint8_t _ID)
{
    ID = _ID;
    _serial.setPort(port);
    _serial.setBaudrate(baudrate);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(200);
    _serial.setTimeout(timeout);

    _serial.open();
    _serial.flushInput();
    cout << "SERIAL OK!" << endl;
}

uint8_t RMDX::vel_cmd(int32_t speed_dps)
{
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = HEADER;
    hex_cmd[1] = ID;
    hex_cmd[2] = DATA_LENGHT;
    hex_cmd[3] = VEL_CMD;

    // set speed
    // cerr << "vel target: " << speed_dps << endl;
    speed_dps *= 100;
    hex_cmd[7] = speed_dps & 0xFF;
    hex_cmd[8] = (speed_dps >> 8) & 0xFF;
    hex_cmd[9] = (speed_dps >> 16) & 0xFF;
    hex_cmd[10] = (speed_dps >> 24) & 0xFF;
    // calculate crc
    unsigned short result = crc16(hex_cmd, sizeof(hex_cmd) - 2);
    hex_cmd[11] = result & 0xFF;
    hex_cmd[12] = (result >> 8) & 0xFF;

    // cerr << "sent hex_cmd:" << endl;
    // for (int i = 0; i < 13; i++)
    // {
    //     printf("%d, %02x\n", i, hex_cmd[i]);
    // }
    _serial.write(hex_cmd, 13);

    // reading
    auto start = chrono::steady_clock::now();
    auto end = chrono::steady_clock::now();

    while (!_serial.available())
    {
        sleep(5);
        end = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(end - start).count() > 100)
            return 2; // wait 100 millis till return error 2
        // double time_taken = chrono::duration_cast<chrono::milliseconds>(end - start).count();
        // cerr << time_taken << endl;
    }
    string line = _serial.read(13);

    // convert string to hex
    for (int i = 0; i < line.size(); i++)
    {
        receive_hex[i] = uint8_t(line[i]);
        printf("%d, %02x\n", i, receive_hex[i]);
    }

    // crc check of received data
    if (crc16(receive_hex, sizeof(receive_hex)) != 0)
    {
        cerr << "crc checking error" << endl;
        return 1;
    }

    return 0;
}

uint8_t RMDX::read_stat()
{
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = HEADER;
    hex_cmd[1] = ID;
    hex_cmd[2] = DATA_LENGHT;
    hex_cmd[3] = READ_STAT;

    // calculate crc
    unsigned short result = crc16(hex_cmd, sizeof(hex_cmd) - 2);
    hex_cmd[11] = result & 0x00FF;
    hex_cmd[12] = result >> 8;

    // cerr << "sent hex_cmd:" << endl;
    // for (int i = 0; i < 13; i++)
    // {
    //     printf("%d, %02x\n", i, hex_cmd[i]);
    // }
    _serial.write(hex_cmd, 13);

    // reading
    auto start = chrono::steady_clock::now();
    auto end = chrono::steady_clock::now();

    while (!_serial.available())
    {
        sleep(5);
        end = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(end - start).count() > 100)
            return 2; // wait 100 millis till return error 2
    }
    string line = _serial.read(13);
    // convert string to hex

    for (int i = 0; i < line.size(); i++)
    {
        receive_hex[i] = uint8_t(line[i]);
        printf("%d, %02x\n", i, receive_hex[i]);
    }

    // crc check of received data
    result = crc16(receive_hex, sizeof(receive_hex));
    if (result != 0)
    {
        cerr << "crc checking error" << endl;
        return 1;
    }

    return 0;
}

uint8_t RMDX::read_multi_turn()
{
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = HEADER;
    hex_cmd[1] = ID;
    hex_cmd[2] = DATA_LENGHT;
    hex_cmd[3] = READ_MULTI_TURN;

    // calculate crc
    unsigned short result = crc16(hex_cmd, sizeof(hex_cmd) - 2);
    hex_cmd[11] = result & 0x00FF;
    hex_cmd[12] = result >> 8;

    // cerr << "sent hex_cmd:" << endl;
    // for (int i = 0; i < 13; i++)
    // {
    //     printf("%d, %02x\n", i, hex_cmd[i]);
    // }
    _serial.write(hex_cmd, 13);

    // reading
    auto start = chrono::steady_clock::now();
    auto end = chrono::steady_clock::now();

    while (!_serial.available())
    {
        sleep(5);
        end = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(end - start).count() > 100)
            return 2; // wait 100 millis till return error 2
    }
    string line = _serial.read(13);
    // convert string to hex

    for (int i = 0; i < line.size(); i++)
    {
        receive_hex[i] = uint8_t(line[i]);
        printf("%d, %02x\n", i, receive_hex[i]);
    }

    // crc check of received data
    result = crc16(receive_hex, sizeof(receive_hex));
    if (result != 0)
    {
        cerr << "crc checking error" << endl;
        return 1;
    }

    return 0;
}

uint8_t RMDX::write_pid(uint8_t curr_kp, uint8_t curr_ki, uint8_t spd_kp, uint8_t spd_ki, uint8_t pos_kp, uint8_t pos_ki)
{
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = HEADER;
    hex_cmd[1] = ID;
    hex_cmd[2] = DATA_LENGHT;
    hex_cmd[3] = SET_PID;

    // set speed
    // cerr << "vel target: " << speed_dps << endl;
    hex_cmd[5] = curr_kp & 0xFF;
    hex_cmd[6] = curr_ki & 0xFF;
    hex_cmd[7] = spd_kp & 0xFF;
    hex_cmd[8] = spd_ki & 0xFF;
    hex_cmd[9] = pos_kp & 0xFF;
    hex_cmd[10] = pos_ki & 0xFF;
    // calculate crc
    unsigned short result = crc16(hex_cmd, sizeof(hex_cmd) - 2);
    hex_cmd[11] = result & 0xFF;
    hex_cmd[12] = (result >> 8) & 0xFF;

    // cerr << "sent hex_cmd:" << endl;
    // for (int i = 0; i < 13; i++)
    // {
    //     printf("%d, %02x\n", i, hex_cmd[i]);
    // }
    _serial.write(hex_cmd, 13);

    // reading
    auto start = chrono::steady_clock::now();
    auto end = chrono::steady_clock::now();

    while (!_serial.available())
    {
        sleep(5);
        end = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(end - start).count() > 100)
            return 2; // wait 100 millis till return error 2
        // double time_taken = chrono::duration_cast<chrono::milliseconds>(end - start).count();
        // cerr << time_taken << endl;
    }
    string line = _serial.read(13);
    // cerr << "received hex:" << endl;
    // convert string to hex

    for (int i = 0; i < line.size(); i++)
    {
        receive_hex[i] = uint8_t(line[i]);
        // printf("%d, %02x\n", i, receive_hex[i]);
    }

    // crc check of received data
    if (crc16(receive_hex, sizeof(receive_hex)) != 0)
    {
        cerr << "crc checking error" << endl;
        return 1;
    }

    return 0;
}

uint8_t RMDX::read_pid()
{
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = HEADER;
    hex_cmd[1] = ID;
    hex_cmd[2] = DATA_LENGHT;
    hex_cmd[3] = GET_PID;

    // calculate crc
    unsigned short result = crc16(hex_cmd, sizeof(hex_cmd) - 2);
    hex_cmd[11] = result & 0x00FF;
    hex_cmd[12] = result >> 8;

    // cerr << "sent hex_cmd:" << endl;
    // for (int i = 0; i < 13; i++)
    // {
    //     printf("%d, %02x\n", i, hex_cmd[i]);
    // }
    _serial.write(hex_cmd, 13);

    // reading
    auto start = chrono::steady_clock::now();
    auto end = chrono::steady_clock::now();

    while (!_serial.available())
    {
        sleep(5);
        end = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(end - start).count() > 100)
            return 2; // wait 100 millis till return error 2
    }
    string line = _serial.read(13);
    // convert string to hex

    for (int i = 0; i < line.size(); i++)
    {
        receive_hex[i] = uint8_t(line[i]);
        printf("%d, %02x\n", i, receive_hex[i]);
    }

    // crc check of received data
    result = crc16(receive_hex, sizeof(receive_hex));
    if (result != 0)
    {
        cerr << "crc checking error" << endl;
        return 1;
    }

    return 0;
}

uint8_t RMDX::release_brake()
{
    memset(hex_cmd, 0, sizeof(hex_cmd));
    hex_cmd[0] = HEADER;
    hex_cmd[1] = ID;
    hex_cmd[2] = DATA_LENGHT;
    hex_cmd[3] = RELEASE_BRAKE;

    // calculate crc
    unsigned short result = crc16(hex_cmd, sizeof(hex_cmd) - 2);
    hex_cmd[11] = result & 0xFF;
    hex_cmd[12] = result >> 8;

    // cerr << "sent hex_cmd:" << endl;
    // for (int i = 0; i < 13; i++)
    // {
    //     printf("%d, %02x\n", i, hex_cmd[i]);
    // }
    _serial.write(hex_cmd, 13);

    // reading
    while (!_serial.available())
    {
        sleep(10);
    }
    string line = _serial.read(13);
    // cerr << "received hex:" << endl;
    // convert string to hex

    for (int i = 0; i < line.size(); i++)
    {
        receive_hex[i] = uint8_t(line[i]);
        // printf("%d, %02x\n", i, receive_hex[i]);
    }

    // crc check of received data
    result = crc16(receive_hex, sizeof(receive_hex));
    if (result != 0)
    {
        cerr << "crc checking error" << endl;
        return 1;
    }

    return 0;
}

// returns value in between max and 0. if value is below min it will set to 0.
int16_t RMDX::limit(int32_t value, int32_t min, int16_t max)
{
    if (value < min && value > -min)
    {
        return 0;
    }
    else if (value > max)
    {
        return max;
    }
    else if (value < -max)
    {
        return -max;
    }
    else
        return value;
}

int16_t RMDX::limit_dif(int32_t value, int32_t prev_value, int32_t min, int16_t max)
{
    int32_t delta = value - prev_value;
    if (delta < min && delta > -min)
    {
        return prev_value;
    }
    else if (delta > max)
    {
        return prev_value + max;
    }
    else if (delta < -max)
    {
        return prev_value - max;
    }
    else
        return value;
}

double RMDX::get_temp()
{
    int8_t temp = receive_hex[4];
    // printf("temp: %d C\n", receive_hex[4]);
    return temp;
}

double RMDX::get_torque()
{
    int16_t torque = receive_hex[5] + (receive_hex[6] << 8);
    // printf("torque: %d *0.01A\n", torque);
    return torque;
}

double RMDX::get_speed_dps()
{
    int16_t speed_dps = receive_hex[7] + (receive_hex[8] << 8);
    // printf("speed: %d dps\n", speed_dps);
    return speed_dps;
}

int32_t RMDX::get_angle()
{
    int32_t angle = receive_hex[9] + (receive_hex[10] << 8);
    // printf("angle: %d deg\n", angle);
    return angle;
}

int32_t RMDX::get_mutli_turn_angle()
{
    int32_t angle = receive_hex[7] + (receive_hex[8] << 8) + (receive_hex[9] << 16) + (receive_hex[10] << 24);
    printf("angle: %d deg\n", angle);
    return angle;
}

uint8_t RMDX::get_kp()
{
    uint8_t kp = receive_hex[7];
    printf("kp: %d \n", kp); // print spd_kp
    return kp;
}

uint8_t RMDX::get_ki()
{
    uint8_t ki = receive_hex[8];
    printf("ki: %d \n", ki); // print spd_ki
    return ki;
}

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum)
{
    cerr << "Caught signal " << signum << endl;
    // cerr << "close Serial.\n";
    // motor._serial.close(); // TODO
    // Terminate program
    exit(signum);
}
