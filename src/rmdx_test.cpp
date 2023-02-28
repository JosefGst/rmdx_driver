#include "rmdx_driver/rmdx.h"

int main(void)
{

    RMDX motor;
    // signal(SIGINT, signal_callback_handler);
    motor.beginn("/dev/motorR", 115200, 0x01);

    motor.vel_cmd(00);

    motor.get_temp();
    motor.get_torque();
    motor.get_speed_dps();
    motor.get_angle();
    motor.sleep(100);

    motor.read_pid();
    motor.get_kp();
    motor.get_ki();

    int32_t prev_vel = 0;

    // main loop
    // for (double i = 0; i < 10000; i++)
    // {

    //     int32_t vel = 200 * sin(i / 100);
    //     // int32_t vel = 1000;
    //     // motor.vel_cmd(vel);

    //     // // motor.read_stat();

    //     // // motor.get_temp();
    //     // // motor.get_torque();
    //     // motor.get_speed_dps();
    //     // // motor.get_angle();

    //     // filtering the vel cmd
    //     int32_t vel_fil = motor.limit_dif(vel, prev_vel, motor.SPEED_DIF_MIN, motor.SPEED_DIF_MAX);
    //     vel_fil = motor.limit(vel_fil, motor.SPEED_MIN, motor.SPEED_MAX);
    //     // cout << "vel: " << vel << "   prev_vel: " << prev_vel << "   vel_fil: " << vel_fil << endl;

    //     if (prev_vel == vel_fil)
    //     {
    //         // cout << "same" << endl;
    //         motor.ERROR_FLAG = motor.read_stat();
    //     }
    //     else
    //     {
    //         motor.ERROR_FLAG = motor.vel_cmd(vel_fil);
    //         prev_vel = vel_fil;
    //     }
    //     motor.get_speed_dps();

    //     while (motor.ERROR_FLAG)
    //     {
    //         motor._serial.close();
    //         motor.sleep(2500);
    //         motor.beginn("/dev/ttyUSB0", 115200, 0x00);
    //         motor.ERROR_FLAG = motor.vel_cmd(00);
    //         i = 0;
    //         motor.ERROR_FLAG = 0;
    //     }

    //     motor.sleep(10);
    // }

    motor.vel_cmd(0);
    motor.sleep(100);
    motor.release_brake();

    cerr << "close Serial main\n";
    motor._serial.close();
    return 0;
}