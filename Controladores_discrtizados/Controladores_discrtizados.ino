//Clase: Sistemas de Control 2
//Sección: 11
//Catedratico: Luis Rivera
//Por: Juan Diego Castillo Amaya -17074 y Hector Alejandro Klée Gonzales - 17118
//Parte 3
#include <SPI.h>
#include "wiring_private.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#define COTA_SUP 3.3
#define COTA_INF 0
//int proceso=0; //se define una variable para seleccionar si se envia
               //un diente de cierra o se utiliza el potenciomentro 
int metodo=0; //se define el metodo que se desea realiza
              //0. sin discretizar
              //1. tustin
              //2. Zero-Pole Matching
              //3. Zero-Orde hold
              //4. Backward euler
//--------------------------------------------Se definen las variables a utilizar en el controlador PID-----------------------------------------
float Uk = 0, Uk_1 = 0, Uk_2 =0;
float Ek = 0, Ek_1 = 0, Ek_2 = 0;
float b0, b1, b2;
float a1, a2;

int Timers;
int diente = 0;
//Enviar la señal de un potenciometro leido por el dac

int frecdiv; //se define la freciencia por la que se va a dividir la frecuencia de 80MHz

const int ss = PD_1; //se espesifica el salve select

const int AnalogInPin = A0; //El pin de netrada del potenciometro (ADC)
const int AnalogOutPin = A1;
int Pot = 0; //se guardara el valor del pot en este lugar
int salida=0;

void setup() {
  pinMode(ss,OUTPUT); //se especifica el pin PB_5 como salida
  SPI.begin(); //se inicializa el SPI
  digitalWrite(ss,HIGH);
//--------------------------------------------Controlador con el sistema sin discretizar---------------------------------------------
  if (metodo==0)
  {
    b0=400;
    b1=65700;
    b2=2430500;
    a1=2485600;
    a2=0;
    Timers=40231;
  }
//--------------------------------------------Controlador con metodo tustin---------------------------------------------------------
  else if (metodo==1)
  {
    b0=35.3277;
    b1=-65.5833;
    b2=30.4366;
    a1=-0.1489;
    a2=-0.5811;
    Timers=1000;
  }
//--------------------------------------------Controlador con metodo Zero-Pole Matching---------------------------------------------------------
  else if (metodo==2)
  {
    b0=163.8685;
    b1=-325.3053;
    b2=161.4457;
    a1=-1.0833;
    a2=0.0833;
    Timers=100;
  }
//--------------------------------------------Controlador con metodo Zero-Order Hold---------------------------------------------------------
  else if (metodo==3)
  {
    b0=440.9373;
    b1=-881.2930;
    b2=440.3559;
    a1=-1.7799;
    a2=0.7799;
    Timers=100;
  }
//--------------------------------------------Controlador con metodo Backward Euler---------------------------------------------------------
  else if (metodo==4)
  {
    b0=353.6824;
    b1=-706.8383;
    b2=353.1561;
    a1=-1.8009;
    a2=0.8009;
    Timers=10;
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
  /*if (proceso == 0)
  {
    for(diente=0;diente<4096;diente++)
    {
      digitalPotWrite(diente, ss);
    }
    for(diente=4095;diente>=0;diente--)
    {
      digitalPotWrite(diente, ss);
    }
  }*/
  //else
  //{
    Pot = analogRead(AnalogInPin)*3.3/4095;
    salida=analogRead(AnalogOutPin)*3.3/4095;
//-----------------------------------------Se implementa el controlador PID en base a la ecuacion de diferencia---------------------------------------
    Ek= Pot - salida;
    Uk= b0*Ek + b1*Ek_1 + b2*Ek_2 -a1*Uk_1 - a2*Uk_2;
    Ek_2=Ek_1;
    Ek_1=Ek;
    Uk_2=Uk_1;
    Uk_1=Uk;
//------------------------------------------Se especifica limite superior para la salida del controlador--------------------------------------------
    if (Uk>COTA_SUP)
    {
      Uk=COTA_SUP;
    }
//------------------------------------------Se especifica limite inferior para la salida del controlador--------------------------------------------
    else if (Uk<COTA_INF)
    {
      Uk=COTA_INF;
    }
//-----------------------------------------Se ingresas la salida del sistema al DAC-------------------------------------------------
    digitalPotWrite(Uk, ss);
  //}
  
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
