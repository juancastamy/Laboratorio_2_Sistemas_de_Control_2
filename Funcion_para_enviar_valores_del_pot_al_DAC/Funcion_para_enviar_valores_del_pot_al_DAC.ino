#include <SPI.h>
#include "wiring_private.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
int proceso=0; //se define una variable para seleccionar si se envia
               //un diente de cierra o se utiliza el potenciomentro 
int Timers;
int diente = 0;
//Enviar la señal de un potenciometro leido por el dac

int frecdiv; //se define la freciencia por la que se va a dividir la frecuencia de 80MHz

const int ss = PD_1; //se espesifica el salve select

const int AnalogInPin = A0; //El pin de netrada del potenciometro (ADC)
int Pot = 0; //se guardara el valor del pot en este lugar

void setup() {
  pinMode(ss,OUTPUT); //se especifica el pin PB_5 como salida
  SPI.begin(); //se inicializa el SPI
  digitalWrite(ss,HIGH);
  if (proceso==0)
  {
    Timers = 10; //se define el tiempo que se desea para que el timer se interrumpa
  }
  else
  {
    Timers = 1000;//se define el tiempo que se desea para que el timer se interrumpa
  }
  frecdiv =(80*Timers);
    configureTimer1A();
  }

void loop() {
  

}

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
  ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, frecdiv); // El último argumento es el CustomValue

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
  if (proceso == 0)
  {
    for(diente=0;diente<4096;diente++)
    {
      digitalPotWrite(diente, ss);
    }
    for(diente=4095;diente>=0;diente--)
    {
      digitalPotWrite(diente, ss);
    }
  }
  else
  {
    Pot = analogRead(AnalogInPin);
    digitalPotWrite(Pot, ss);
  }
  
}
int digitalPotWrite(int value, int slave_select) {
  //se apaga el pin para seleccionar el chip a utilizar
  digitalWrite(slave_select,LOW);
  byte primero = (byte)((value>>8) & 0xFF);// con el operador>> se realiza un shift, en este caso de 8 bits por 
                                           // lo que se descartan los 8 bits menos significativos y se realiza un 
                                           // and con el valor 0xFF. Esto permite que los bits en 1 se mantengan 
                                           //como 1 y los que estan en 0 se coloquen como 0
  byte segundo = (byte)(value & 0xFF); //este realiza un and con los primero 8 bits del valor leido y reliza un and
                                       //esto con el proposito de que los primero 8 bits del valor values se asignen
                                       //en la variable segundo
  SPI.transfer(primero);//se escribe los primero 4 bits de value
  SPI.transfer(segundo);//se escriben los primero 8 bits de values
  // take the SS pin high to de-select the chip:
  digitalWrite(slave_select,HIGH);
}
