/* 
 * File:   Postlab9.c
 * Author: Saby Andrade 20882
 *Descripcion:Utilizando otro canal analógico realice la conversión ADC y realice una tercera
señal de PWM manualmente . 
. El contador en cero deberá poner una
salida en alto y cuándo este llegue al valor seteado por un tercer potenciómetro
se deberá de poner en cero la salid
 */

// Configuracion 1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

//Configuracion 2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

/* 
 *Librerías
 */
#include <xc.h>
#include <stdint.h>

/* 
 * Constantes
 */
//Valor del oscilador
#define _XTAL_FREQ 4000000
//Valor del timer 0 (tiempo de retardo)
#define _tmr0_value 250   

/* 
 * Variables
 */
uint8_t contar;
uint8_t potenciometro3;


/* 
 *Prototipo de Funciones
 */
void setup(void);

/* 
 *Interrupciones
 */
void __interrupt() isr (void)
{   
    //Interrupcion del ADC
    if (PIR1bits.ADIF)  
    {
        //Canal 0
         if (ADCON0bits.CHS == 0){
             //
            CCPR1L = (ADRESH>>1)+30;
            CCP1CONbits.DC1B1 = ADRESH & 0b01;
            CCP1CONbits.DC1B0 = ADRESL >> 7;
         }
           
         //Canal 1
        else if (ADCON0bits.CHS == 1){

            CCPR2L = (ADRESH>>1)+45;
            CCP2CONbits.DC2B1 = ADRESH & 0b01;
            CCP2CONbits.DC2B0 = ADRESL >> 7;
        }
        else
            //El valor que nos entrega el tercer pot se asignará aqui
            potenciometro3 = ADRESH;
        PIR1bits.ADIF = 0;   
    }

    //Interrupcion del timer 0
    if(T0IF == 1){            
        INTCONbits.T0IF = 0;
 //Incrementa el contador
        contar ++;   
 //Valor del timer0
        TMR0 = _tmr0_value;
        
//Es menor el contador que el potenciometro 3, se encenderá el led, en el puerto B
        if (contar < potenciometro3) 
            PORTBbits.RB0 = 1;
        else
//Al no pasar eso estará apagado el led, en el Puerto B
            PORTBbits.RB0 = 0;
        INTCONbits.T0IF = 0;
    }    
}

/* 
 * Ciclo Principal
 */

void main(void) {
    //Configuración inicial
    setup();
    while(1){
    if (ADCON0bits.GO == 0) {
        if (ADCON0bits.CHS == 0)  
            //Primer Canal, primer potenciometro
            ADCON0bits.CHS = 1;
        else if (ADCON0bits.CHS == 1)
            
            //Segundo Canal, segundo potenciometro
            ADCON0bits.CHS = 2;
        else
            //Tercer canal, tercer potenciometro, servirá para convertir el valor que nos entregue el tercer potenciometro
            ADCON0bits.CHS = 0;
        //El tiempo del delay
        __delay_us(1000);
        ADCON0bits.GO = 1;    
        }   
    }        
}

/* 
 * Configuración
 */
void setup (void){
    // Configuración de los puertos
    //AN0 Como entradas analógicas
    ANSEL = 0b00000111;       
    ANSELH = 0;
    
    //Puerto A0 como salida
    TRISAbits.TRISA0 = 0b00000111;
    //Puerto A1 como salida
    TRISAbits.TRISA1 = 0b00000111;
    //Puerto A2 como salida
    TRISAbits.TRISA2 = 0b00000111; 
    //Puerto A3 como salida
    TRISAbits.TRISA3 = 0b00000111; 
    //Puerto A4 como salida
    TRISAbits.TRISA4 = 0b00000111;
    //Puerto A5 como salida
    TRISAbits.TRISA5 = 0b00000111;
    //Puerto A6 como salida
    TRISAbits.TRISA6 = 0b00000111; 
    //Puerto A7 como salida
    TRISAbits.TRISA7 = 0b00000111;    
    
    //Se limpia el puerto A0
    PORTAbits.RA0 = 0;
    //Se limpia el puerto A1
    PORTAbits.RA1 = 0;
    //Se limpia el puerto A2
    PORTAbits.RA2 = 0;
    //Se limpia el puerto A3
    PORTAbits.RA3 = 0;
    //Se limpia el puerto A4
    PORTAbits.RA4 = 0;
     //Se limpia el puerto A5
    PORTAbits.RA5 = 0;
    //Se limpia el puerto A6
    PORTAbits.RA6 = 0;
    //Se limpia el puerto A7
    PORTAbits.RA7 = 0;
    
    //Puerto B0 como salida
    TRISBbits.TRISB0 = 0;
    //Puerto B1 como salida
    TRISBbits.TRISB1 = 0;
    //Puerto B2 como salida
    TRISBbits.TRISB2 = 0; 
    //Puerto B3 como salida
    TRISBbits.TRISB3 = 0; 
    //Puerto B4 como salida
    TRISBbits.TRISB4 = 0;
    //Puerto B5 como salida
    TRISBbits.TRISB5 = 0;
    //Puerto B6 como salida
    TRISBbits.TRISB6 = 0; 
    //Puerto B7 como salida
    TRISBbits.TRISB7 = 0; 
    
    //Se limpia el puerto B0
    PORTBbits.RB0 = 0;
    //Se limpia el puerto B1
    PORTBbits.RB1 = 0;
    //Se limpia el puerto B2
    PORTBbits.RB2 = 0;
    //Se limpia el puerto B3
    PORTBbits.RB3 = 0;
    //Se limpia el puerto B4
    PORTBbits.RB4 = 0;
     //Se limpia el puerto B5
    PORTBbits.RB5 = 0;
    //Se limpia el puerto B6
    PORTBbits.RB6 = 0;
    //Se limpia el puerto B7
    PORTBbits.RB7 = 0;
    
    
    // Configuración del oscilador
    OSCCONbits.IRCF = 0b0110;  // IRCF <2:0> -> 111 4 MHz
    OSCCONbits.SCS = 1;         // Oscilador interno

    // Configuración del TMR0
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.T0SE = 0;
    OPTION_REGbits.PSA = 0;     //Un TMR0 con un Prescaler 1:256
    OPTION_REGbits.PS2 = 0;
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS0 = 0;  
      // Reiniciamos el TMR0 a 217 para tener un retardo de 5ms
    TMR0 = _tmr0_value;  
        
    //Se limpian las interrupciones del Timer0
    INTCONbits.T0IE = 1;      
    //Se limpia la bandera de interrupcion del timer0
    INTCONbits.T0IF = 0;    
        
    //COnfiguración del ADC
     // ADCS <1:0> -> 10 FOSC/32
    ADCON0bits.ADCS = 0b01;    
    // CHS  <3:0> -> 0000 AN0
    ADCON0bits.CHS = 0;   
    //Justificando a la izquierda
    ADCON1bits.ADFM = 0;  
    //Referencia en VDD
    ADCON1bits.VCFG0 = 0;     
    //Referencia en VSS
    ADCON1bits.VCFG1 = 0;     
    
    
    // Configuración del PWM
    // RC1 -> CCP2 como entrada; // RC2 -> CCP1 como entradas
    TRISCbits.TRISC2 = 1;     
    TRISCbits.TRISC1 = 1;    
    //Valor del Periodo del timer2
    PR2 = 155;               
    //Salida Simple
    CCP1CONbits.P1M = 0;       
    //Se asigna el modo de PW1
    CCP1CONbits.CCP1M = 0b1100; 
    //Se asigna el modo de PW2
    CCP2CONbits.CCP2M = 0b1100; 
           
    // Valor inicial del duty cycle
    CCPR1L = 0x0F;             
    //Configuracion bits de menor importancia (menos significativos)
    CCP1CONbits.DC1B = 0;     
    
    //Valor inicial del duty cycle
    CCPR2L = 0x0F;             
        //Configuracion bits de menor importancia (menos significativos)
    CCP2CONbits.DC2B0 = 0;
    
    // Configuración del TIMER2
    //Flag del Timer2 en 0
    PIR1bits.TMR2IF = 0;      
    //Prescaler 1:16
    T2CONbits.T2CKPS = 0b11;  
    //Se enciende el Timer2
    T2CONbits.TMR2ON = 1;   
    
    //Se espera la interrupcion del Timer2
    while (PIR1bits.TMR2IF == 0);
    PIR1bits.TMR2IF = 0;
    
    // RC2 -> CCP1 como salida del PWM
    TRISCbits.TRISC2 = 0;       
    // RC1 -> CCP2 como salida
    TRISCbits.TRISC1 = 0;    
    
    //Configuración de las interrupciones
    //Se habilitan las interrupciones del ADC
    PIE1bits.ADIE = 1;         
    //Banderas del ADC en 0
    PIR1bits.ADIF = 0; 
    //Se habilitan las interrupciones de los puertos
    INTCONbits.PEIE = 1;      
    //Se habilitan las interrupciones globales
    INTCONbits.GIE = 1;      
    //Valor del tiempo del delay
    __delay_us(50);
    //Se enciende ADC
    ADCON0bits.ADON = 1;       
    //Se regresa
    return;  
}