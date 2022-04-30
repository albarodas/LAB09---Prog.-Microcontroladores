/* 
 * File:   LAB_09.c
 * Author: ALBA RODAS
 *
 * Created on 24 de abril de 2022, 12:49 PM
 */
// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIGURATION WORDS 1
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

// CONFIGURATION WORDS 1
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

/*------------------------------------------------------------------------------
 * LIBRERIAS 
 ------------------------------------------------------------------------------*/
// INCLUIMOS LIBRERIAS A UTILIZAR, SEGUN FUNCIONES
#include <xc.h>
#include <stdint.h>
// DEFINIMOS UN OSCILADOR DE 8MHz
#define _XTAL_FREQ 8000000
/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t counter;                   // VARIABLE PARA CONTADOR
uint8_t adresh_variable;           // VARIABLE PARA MANIPULAR ADRESH
/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void config_inst_outs(void);
void config_clk(void);
void config_tmr0(void);
void config_adc(void);
void config_pwm_motor2(void);
void config_interrupciones(void);
void config_pwm_motor1(void);
/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void)
{
    if(PIR1bits.ADIF)                   // BANDERA = ON --> SIGO ADELANTE
    {
        if(ADCON0bits.CHS == 5)         // ELEJÍ EL CANAL 5
        {
            CCPR1L = (ADRESH>>1)+120;
            CCP1CONbits.DC1B1 = ADRESH & 0b01;
            CCP1CONbits.DC1B0 = (ADRESH>>7);
        }else if(ADCON0bits.CHS == 6)   // ELEJÍ EL CANAL 6
        {
            CCPR2L = (ADRESH>>1)+120;   // VALOR == 124
            CCP1CONbits.DC1B1 = ADRESH & 0b01;
            CCP1CONbits.DC1B0 = (ADRESH>>7);
        }else                           // ELEJÍ EL CANAL 5
        {
            adresh_variable = ADRESH;
        }
        PIR1bits.ADIF = 0;
    }else if(INTCONbits.T0IF)    // BANDERA = ON --> SIGO ADELANTE
    {
        counter = counter + 1;
        
        if (counter >= adresh_variable)
        {
            PORTDbits.RD0 = 0;
        } else
        {
            PORTDbits.RD0 = 1;
        }
        
        TMR0 = 225;
        INTCONbits.T0IF = 0;   
    }
}
/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) 
{
    config_inst_outs();
    config_clk();
    config_adc();
    config_pwm_motor1();
    config_pwm_motor2();
    config_interrupciones();
    config_tmr0();
    
    ADCON0bits.GO = 1;                       // CONVERSIÓN = ACTIVADA
    while(1)
    {
        if(ADCON0bits.GO == 0)              // ULTIMA CONVERSION TERMINADA == CONTINUE A LA SIGUIENTE LINEA
        {
            if(ADCON0bits.CHS == 5)         // ULTIMO = CANAL 5 --> PASO AL 6
                ADCON0bits.CHS = 6;
            else if(ADCON0bits.CHS == 6)    // ULTIMO = CANAL 6 --> PASO AL 7
                ADCON0bits.CHS = 7;
            else                            // NOT 5, 6, 7 --z VUELVO AL CANAL 5
                ADCON0bits.CHS = 5;     
            __delay_us(50);                 // OJO AL DELAL, PUEDE SER 40us, pero con 50us PROTEUS no falla.
            ADCON0bits.GO = 1;              // INICIO NUEVAMENTE LA CONVERSION
        }
        
    }
}
/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void config_inst_outs(void)     // CONFIGURAMOS SALIDAS Y ENTRADAS
{
    ANSEL   =   0b11100000;     // ANALOGIC PORTS = AN5, AN6 y AN7
    ANSELH  =   0;              // DEFINO PUERTOS DIGITALES
    
    TRISD   =   0;              // DEFINO PORTD COMO SALIDA
    
    // CLRF AL PORTD PARA EVITAR ERRORES DE PERFORMANCE
    PORTD   =   0;              
    return;
}

void config_clk(void)           // CONFIGURACION DEL RELOJ
{
    OSCCONbits.IRCF2 = 1;       // DEFINIMOS UN OSCILADOR DE 8MHz
    OSCCONbits.IRCF1 = 1;       // 8MHz = 111 -> DEL BIT 2:0
    OSCCONbits.IRCF0 = 1;       
    OSCCONbits.SCS = 1;         // ACTIVO RELOJ INTERNO
    return;
}
//CONFIGURACION TMR0
void config_tmr0(void)
{
    // generar  una interrupción  y  aumentar  contador.
    OPTION_REGbits.T0CS = 0;  // BIT 5 --> EN 0 PARA = FOSC/4
    OPTION_REGbits.T0SE = 0;  // BIT 4 --> EN 0 PARA = Increment on low-to-high transition on T0CKI pin
    OPTION_REGbits.PSA = 0;   // BIT 3 --> EN 0 PARA = ASIGNAR VALOR DE PRESCALER AL MODULO DEL TIMER0
    OPTION_REGbits.PS2 = 0;   // bits 2-0  PS2:PS0: Prescaler Rate Select bits
    OPTION_REGbits.PS1 = 0;   // PS <2:0> = 000 --> TMR0 RATE = 1:2 --> WDT RATE = 1:1
    OPTION_REGbits.PS0 = 0;
    TMR0 = 225; // VALOR DEL PRESCALER = 255
    return;
}
// CONFIGURACION ADC
void config_adc(void)
{
     // CONFIGURAMOS REFERENCIAS INTERNAS:
    ADCON1bits.ADFM = 0;        // JUSTIFICADO A LA IZQUIERDA
    ADCON1bits.VCFG0 = 0;       // REFERENCIA --> VDD
    ADCON1bits.VCFG1 = 0;       // REFERENCIA --> VSS
    
    // CONFIGURAMOS QUE CANAL ANALOGICO SERA NUESTRA ENTRADA:
    ADCON0bits.ADCS = 0b10;     // UTILIZAMOS FOSC/8
    ADCON0bits.CHS = 5;         // ELIJO UTILIZAR EL CANAL 5, DE LOS 14 CANALES DISPONIBLES
    ADCON0bits.ADON = 1;        // ADC == ON
    __delay_us(50);             // DELAY DE 50us
    return;
}
// CONFIGURACION PWM --> MOTOR 1
void config_pwm_motor1(void)
{
    // EL PWM GENERA SEÑALES DE FRECUENCIA Y CICLO DE TRABAJO. MODULO CCP Y TIMER2, 
    // TRABAJAN JUNTOS PARA FORMAR UN MODULADOR DE ANCHO DE PULSO, EN BASE A UN PERÍODO.
    // PERIODO A CARGO DEL TMR2 Y PR2
    TRISCbits.TRISC2 = 1;           // RC2/CCP1 como entrada
    PR2 = 250;                      // Periodo
    CCP1CONbits.P1M = 0;
    CCP1CONbits.CCP1M = 0b1100;
    
    CCPR1L = 0b1111;                //  Duty cycle  
    CCP1CONbits.DC1B = 0;
    
    PIR1bits.TMR2IF = 0;            //TMR 2 --> 0 = No Timer2 to PR2 match occurred
    T2CONbits.T2CKPS = 0b11;        // T2CKPS<1:0> --> Timer2 Clock Prescale Select bits
    // T2CKPS<1:0>: Timer2 Clock Prescale Select bits ==> 1x = Prescaler is 16
    T2CONbits.TMR2ON = 1;          // TMR2ON: Timer2 On bit
                                    // 1 = Timer2 is on
    // The completion of a full PWM cycle
    //is indicated by the TMR2IF bit of the PIR1 register
    //being set as the second PWM period begins
    while(PIR1bits.TMR2IF == 0);
    PIR1bits.TMR2IF = 0;             // No Timer2 to PR2 match occurred
    
    TRISCbits.TRISC2 = 0;           // RC2 --> SALIDA
}
// CONFIGURACION PWM --> MOTOR 2
void config_pwm_motor2(void)
{
    TRISCbits.TRISC1 = 1;           // RC1/CCP2 --> LUEGO LAS IGUALAMOS A CERO.
    CCP2CONbits.CCP2M = 0b1100;     // 11xx = PWM mode.
    
    CCPR2L = 0b1111;                // 11xx = PWM mode. 
    CCP2CONbits.DC2B0 = 0;          // DC2B<1:0> --> PWM Duty Cycle Least Significant bits
    CCP2CONbits.DC2B1 = 0;
    // PWM mode:
    // These bits are the two LSbs of the PWM duty cycle. The eight MSbs are found in CCPR2L
    TRISCbits.TRISC1 = 0;           // RC1 --> SALIDA; IGUALAR A CERO
}
// CONFIGURACION INTERRUPCIONES
void config_interrupciones(void)      
{
    INTCONbits.GIE  = 1;         // ACTIVO LAS INTERRUPCIONES GLOBALES, PARA PODER UTILIZAR INTERRUPCIONES.
    INTCONbits.PEIE = 1;        // INT. PERIFERICAS == ON
    INTCONbits.T0IE = 1;        // INTERRUPCIONES Tmr0 == ON
    INTCONbits.TMR0IF = 0;
    PIE1bits.ADIE = 1;          // ACTIVO INTERRUPCIONES PARA EL TMR0.
    PIR1bits.ADIF = 0;          // APAGO LA BANDERA DEL TMR0.
    return;
}
/*------------------------------------------------------------------------------
 * TABLA
 ------------------------------------------------------------------------------*/
// TRADUZCO VALORES DEL 0 AL 9, BINARIO --> DECIMAL
// NO SE NECESITA PARA ESTE LAB