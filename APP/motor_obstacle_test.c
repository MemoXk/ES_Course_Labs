#include "motor_obstacle_test.h"
#include "../HAL/MOTOR/MOTOR_interface.h"
#include "../HAL/SOFT_PWM/SOFT_PWM_interface.h"
#include "../HAL/ULTRASONIC/ULTRASONIC_interface.h"

/*
 * Drive forward at 25% software PWM speed.
 * Stop when front ultrasonic detects object < 20 cm.
 * Resume automatically once path is clear.
 *
 * Soft PWM pin : RD0
 * Motor pins   : RD4-RD7
 * Ultrasonic   : RB0 (TRIG), RB1 (ECHO)
 */
void MOTOR_OBSTACLE_Test(void)
{
    u16 dist;

    ULTRASONIC_Init();
    SOFT_PWM_Init();
    MOTOR_Init();
    MOTOR_Forward();

    while(1)
    {
        dist = ULTRASONIC_GetDistance(ULTRASONIC_FRONT);

        if(dist < ULTRASONIC_STOP_THRESHOLD)
        {
            SOFT_PWM_Stop();
            MOTOR_Stop();

            while(ULTRASONIC_GetDistance(ULTRASONIC_FRONT) < ULTRASONIC_STOP_THRESHOLD)
            {
                Delay_ms(50);
            }

            MOTOR_Forward();
        }

        SOFT_PWM_Tick();    /* one 10ms PWM period at 25% duty */
    }
}
