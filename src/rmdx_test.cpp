#include "rmdx_driver/rmdx.h"

int main(void)
{

    RMDX motor;
    // signal(SIGINT, signal_callback_handler);
    motor.beginn("/dev/motorR", 115200, 0x01);

    motor.read_multi_turn_offset();
    printf("multi turn offset\n");
    motor.get_mutli_turn_angle();

    motor.vel_cmd(00);

    int32_t prev_vel = 0;
    int prev_angle = 0;
    int curr_angle = motor.get_angle();
    int overflow_count = 0;
    int abs_angle = 0;

    motor.get_temp();
    motor.get_torque();
    motor.get_speed_dps();
    motor.get_angle();
    motor.sleep(100);

    motor.read_pid();
    motor.get_kp();
    motor.get_ki();

    uint8_t curr_kp = 100;
    uint8_t curr_ki = 50;
    uint8_t spd_kp = 50;
    uint8_t spd_ki = 5;
    uint8_t pos_kp = 100;
    uint8_t pos_ki = 1;

    motor.write_pid(curr_kp, curr_ki, spd_kp, spd_ki, pos_kp, pos_ki);
    // motor.get_kp();
    // motor.get_ki();
    motor.read_multi_turn();
    motor.get_mutli_turn_angle();

    // main loop
    for (double i = 0; i < 10000; i++)
    {

        int32_t vel = 500 * sin(i / 1000);
        // int32_t vel = 1000;
        motor.vel_cmd(vel);

        // motor.read_stat();

        // motor.get_temp();
        // motor.get_torque();
        motor.get_speed_dps();
        // motor.get_angle();

        // filtering the vel cmd
        int32_t vel_fil = motor.limit_dif(vel, prev_vel, motor.SPEED_DIF_MIN, motor.SPEED_DIF_MAX);
        vel_fil = motor.limit(vel_fil, motor.SPEED_MIN, motor.SPEED_MAX);
        // cout << "vel: " << vel << "   prev_vel: " << prev_vel << "   vel_fil: " << vel_fil << endl;

        if (prev_vel == vel_fil)
        {
            // cout << "same" << endl;
            motor.ERROR_FLAG = motor.read_stat();
        }
        else
        {
            motor.ERROR_FLAG = motor.vel_cmd(vel_fil);
            prev_vel = vel_fil;
        }
        motor.vel_cmd(vel);
        motor.get_speed_dps();

        // motor.read_multi_turn();
        // motor.get_mutli_turn_angle();
        prev_angle = curr_angle;
        curr_angle = motor.get_angle();
        // printf("prev_angle: %d\n", prev_angle);
        printf("curr_angle: %d\n", curr_angle);
        int angle_del = curr_angle - prev_angle;
        printf("angle_del: %d\n", angle_del);

        if (angle_del > 1000)
        {
            printf("data overflow\n");
            angle_del -= 7280;
        }
        else if (angle_del < -1000)
        {
            printf("data overflow\n");
            angle_del += 7280;
        }
        abs_angle += angle_del;
        printf("abs_angle: %d\n", abs_angle);

        while (motor.ERROR_FLAG)
        {
            motor._serial.close();
            motor.sleep(2500);
            motor.beginn("/dev/motorR", 115200, 0x00);
            motor.ERROR_FLAG = motor.vel_cmd(00);
            i = 0;
            motor.ERROR_FLAG = 0;
        }

        motor.sleep(10);
    }

    motor.vel_cmd(0);
    motor.sleep(100);
    motor.release_brake();

    cerr << "close Serial main\n";
    motor._serial.close();
    return 0;
}