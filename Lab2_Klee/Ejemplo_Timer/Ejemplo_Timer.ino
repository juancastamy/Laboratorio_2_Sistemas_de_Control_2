// Ejemplo_Timer
// Código original tomado de:
//      https://henryforceblog.wordpress.com/2015/05/02/blink-example-using-timer-on-a-tiva-launchpad/
// Modificado por Luis Alberto Rivera

#include "wiring_private.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"

#define LED RED_LED

volatile uint8_t state = 0;

void setup(){
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);   
  configureTimer1A(); // llamado a configuración del timer.
}

void loop(){
  // El loop principal no hace nada. Todo se realiza en el handler de la interrupción, cuando
  // se vence el timer.
}

// Función que configura el timer (1A en este ejemplo)
void configureTimer1A(){
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // Enable Timer 1 Clock
  ROM_IntMasterEnable(); // Enable Interrupts
  ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // Configure Timer Operation as Periodic
  
  // Configure Timer Frequency
  // El tercer argumento ("CustomValue") de la siguiente función debe ser un entero, no un float.
  // Ese valor determina la frecuencia (y por lo tanto el período) del timer.
  // La frecuecia está dada por: MasterClock / CustomValue
  // En el Tiva C, el MasterClock es de 80 MHz.
  // Ejemplos:
  // Si se quiere una frecuencia de 1 Hz, el CustomValue debe ser 80000000. 80MHz/80M = 1 Hz
  // Si se quiere una frecuencia de 1 kHz, el CustomValue debe ser 80000. 80MHz/80k = 1 kHz
  ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, 80000000); // El último argumento es el CustomValue

  // Al parecer, no hay función ROM_TimerIntRegister definida. Usar la de memoria FLASH
  // El prototipo de la función es:
  //    extern void TimerIntRegister(uint32_t ui32Base, uint32_t ui32Timer, void (*pfnHandler)(void));
  // Con el tercer argumento se especifica el handler de la interrupción (puntero a la función).
  // Usar esta función evita tener que hacer los cambios a los archivos internos de Energia,
  // sugeridos en la página de donde se tomó el código original.
  TimerIntRegister(TIMER1_BASE, TIMER_A, &Timer1AHandler);
  
  ROM_IntEnable(INT_TIMER1A);  // Enable Timer 1A Interrupt
  ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // Timer 1A Interrupt when Timeout
  ROM_TimerEnable(TIMER1_BASE, TIMER_A); // Start Timer 1A
}

// Handler (ISR) de la interrupción del Timer
void Timer1AHandler(void){
  //Required to launch next interrupt
  ROM_TimerIntClear(TIMER1_BASE, TIMER_A);
  
  state ^= 0x01; // Toggle led state (usa operación XOR por cada bit: state = state XOR 00000001b)
  digitalWrite(LED, state); // Blink
}
