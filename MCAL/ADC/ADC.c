#include "ADC_Interface.h"

/* =========================================================
   ADC_Init
========================================================= */

void ADC_Init(void)
{
    /* Configure port pins and result justification */
    ADCON1 = ADC_PORT_CONFIG;

    /* Set ADC clock, select channel 0, turn ADC on */
    ADCON0 = (u8)((ADC_CLOCK_SEL << 6) | (1U << ADON_BIT));

    /* Clear the ADC interrupt flag */
    CLR_BIT(PIR1, ADIF_BIT);
}

/* =========================================================
   ADC_SelectChannel
========================================================= */

void ADC_SelectChannel(u8 Channel)
{
    /* Clear CHS bits then set the requested channel */
    ADCON0 = (u8)((ADCON0 & (u8)(~ADC_CHANNEL_MASK)) | ((Channel & 0x07U) << 3));
}

/* =========================================================
   ADC_Read
   Returns 10-bit result (right-justified)
========================================================= */

u16 ADC_Read(u8 Channel)
{
    u8 delay;

    ADC_SelectChannel(Channel);

    /* Acquisition delay — allow capacitor to charge */
    for(delay = 0; delay < ADC_ACQUISITION_DLY; delay++) { ; }

    /* Start conversion */
    SET_BIT(ADCON0, GO_BIT);

    /* Wait for conversion to complete (GO_BIT cleared by hardware) */
    while(GET_BIT(ADCON0, GO_BIT)) { ; }

    /* Clear interrupt flag */
    CLR_BIT(PIR1, ADIF_BIT);

    /* Return right-justified 10-bit result */
    return (u16)(((u16)ADRESH << 8) | (u16)ADRESL);
}

/* =========================================================
   ADC_Disable
========================================================= */

void ADC_Disable(void)
{
    CLR_BIT(ADCON0, ADON_BIT);
}
