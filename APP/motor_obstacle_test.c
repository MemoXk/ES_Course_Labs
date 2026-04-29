#include "motor_obstacle_test.h"
#include "../HAL/MOTOR/MOTOR_interface.h"
#include "../HAL/ULTRASONIC/ULTRASONIC_interface.h"
#include "../MCAL/PWM/PWM_Interface.h"

#define DRIVE_SPEED_PERCENT     25u

/*
 * Drive forward at 25% PWM speed.
 * Stop immediately when front ultrasonic detects object < 20 cm.
 * Resume automatically once the path is clear again.
 *
 * Hardware note: remove ENA and ENB jumpers from the L298N and
 * connect PIC RC2 (CCP1 PWM output) to both ENA and ENB.
 */
void MOTOR_OBSTACLE_Test(void)
{
    u16 front_dist;

    ULTRASONIC_Init();

    PWM_Init();
    PWM_SetDutyCycle(DRIVE_SPEED_PERCENT);
    PWM_Start();

    MOTOR_Init();
    MOTOR_Forward();

    while(1)
    {
        front_dist = ULTRASONIC_GetDistance(ULTRASONIC_FRONT);

        if(front_dist < ULTRASONIC_STOP_THRESHOLD)
        {
            MOTOR_Stop();
            PWM_Stop();

            /* Wait until obstacle clears */
            while(ULTRASONIC_GetDistance(ULTRASONIC_FRONT) < ULTRASONIC_STOP_THRESHOLD)
            {
                Delay_ms(50);
            }

            PWM_SetDutyCycle(DRIVE_SPEED_PERCENT);
            PWM_Start();
            MOTOR_Forward();
        }
    }
}
