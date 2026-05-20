#include "DELAY_Interface.h"

/*
 * Keep compiler-provided wait primitives inside this MCAL module.
 * Application and HAL code call DELAY_us()/DELAY_ms() instead of
 * depending directly on XC8 builtins.
 */
#if defined(EMBEDDED_HOST_LINT)
#define DELAY_BACKEND_US_1()   do { } while(0)
#define DELAY_BACKEND_MS_1()   do { } while(0)
#else
#define DELAY_BACKEND_US_1()   __delay_us(1)
#define DELAY_BACKEND_MS_1()   __delay_ms(1)
#endif

void DELAY_us(u16 delay_us)
{
    while(delay_us > 0U)
    {
        DELAY_BACKEND_US_1();
        delay_us--;
    }
}

void DELAY_ms(u16 delay_ms)
{
    while(delay_ms > 0U)
    {
        DELAY_BACKEND_MS_1();
        delay_ms--;
    }
}
