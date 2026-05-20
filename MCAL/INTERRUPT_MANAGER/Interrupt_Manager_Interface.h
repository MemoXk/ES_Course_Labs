#ifndef INTERRUPT_MANAGER_INTERFACE_H
#define INTERRUPT_MANAGER_INTERFACE_H

/*
 * The PIC/XC8 interrupt entry syntax is intentionally hidden here.
 * Driver code names the vector through INTERRUPT_MANAGER_ISR_ENTRY()
 * so only the interrupt-manager boundary knows about compiler syntax.
 */
#if defined(EMBEDDED_HOST_LINT)
#define INTERRUPT_MANAGER_ISR_ENTRY(name)  void name(void)
#else
#define INTERRUPT_MANAGER_ISR_ENTRY(name)  void __interrupt() name(void)
#endif

#endif
