// Sleep and watch for light going on and off

#include <htc.h>
// #define _XTAL_FREQ 500000  // OSSCON 500kHz with internal oscillator defualt value is 500kHz
#define _XTAL_FREQ 31000  // OSSCON 31kHz 
                    
#define ROOM_LIGHT_ON 1
#define ROOM_LIGHT_OFF 0
#define ROOM_LIGHT_VALUE  65000 // values smaller than this - light is on
#define ROOM_LIGHT_COUNT_ON 1 //how long light has to stay as it is to be seen as change
#define ROOM_LIGHT_COUNT_OFF 6 //how long light has to stay as it is to be seen as change

#define DIMM_LIGHT_VALUE 50000

#define CHECK_FREQUENCY_1S 0b01010
#define CHECK_FREQUENCY_05S 0b01001
#define CHECK_FREQUENCY_2S 0b01011

#define ARDUINO_INTERUPT_ROOM_LIGHT_ON  10
#define ARDUINO_INTERUPT_ROOM_LIGHT_OFF  40
#define ARDUINO_INTERUPT_SWITCH 11

unsigned light_on = 0;
unsigned switch_interrupt= 0;

void Enable_PWM_LED_Dimmer_RA5();
void Disable_PWM_LED_Dimmer_RA5();
void Set_PWM_RA5(int duty);
void ArduinoInterupt(unsigned short time);
unsigned int ReadLdr();


void interrupt isr() {
    //reset the interrupt flag

    if (INTCONbits.INTE && INTCONbits.INTF) {
        INTCONbits.INTF = 0;
        
        switch_interrupt=1;

        if (light_on != 1) {
            light_on = 1;
        } else {
          light_on = 0;
        }

    }
}



void main() {

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)
    
#pragma config WDTE = SWDTEN       // Watchdog Timer Enable (WDT enabled) 

    
    OSCCONbits.IRCF=0b000; // (31khz)
    OSCCONbits.SCS=0b10; 
    
    APFCON0bits.RXDTSEL=1; // 1 = RX/DT function is on RA1 (otherwise RC5/RC4 will not work as I/O)
    APFCON1bits.CCP2SEL=0b1; // 1 = CCP2/P2A function is on RA5    

    WDTCONbits.WDTPS = CHECK_FREQUENCY_2S; // 
    
    PORTA = 0b00000000;
    LATA = 0b00000000; // all input
    ANSELA = 0b00000000; // no analog read

    PORTC = 0b00000000;
    LATC = 0b00000000; // all input
    ANSELC = 0b00000000; // no analog read

    // Define as Output
    TRISAbits.TRISA5 = 0b0; // RA5: PWM out, to dimm LED for light
    TRISCbits.TRISC4 = 0b0; // RC4: Digital Out, Power for LDR
    TRISCbits.TRISC5 = 0b0; // RC5: Digital Out, Interrupt to Arduino
    
    // Define as Input
    TRISAbits.TRISA2 = 0b1; // RA2: Digital IN, from cap-switch to create interrupt in PIC

    // Define Analog read AN3
    TRISCbits.TRISC3 = 0b1; // RC3: Analog In, Measure LDR    
        
     // Configure Interrupt
    INTCONbits.INTF = 0; //reset the external interrupt flag
    OPTION_REGbits.INTEDG = 1; //interrupt on the rising edge
    INTCONbits.INTE = 1; //enable the external interrupt
    INTCONbits.GIE = 1; //set the Global Interrupt Enable

    // Configure Analog Converter to read LDR value
    ADCON0bits.CHS= 0b00111; //Analog Channel Select bits, AN7
    ADCON1bits.ADCS = 0b101; // use Fosc/16 for clock source
    ADCON1bits.ADNREF=0b0; //negative voltage ref to VSS (GND)
    ADCON1bits.ADPREF=0b00; //positive voltage ref to VDD
    
    // Say Hello
    TRISAbits.TRISA5 = 0b0; // RA1 = PWM2 register : PWM out, to dimm LED
    LATAbits.LATA5 = 0b1; __delay_ms(1000); LATAbits.LATA5 = 0b0;
    TRISAbits.TRISA5 = 0b1; // RA1 = PWM2 register : PWM out, to dimm LED

    // Say Hello
    TRISAbits.TRISA5 = 0b0; // RA1 = PWM2 register : PWM out, to dimm LED
    LATAbits.LATA5 = 0b1; __delay_ms(1000); LATAbits.LATA5 = 0b0;
    TRISAbits.TRISA5 = 0b1; // RA1 = PWM2 register : PWM out, to dimm LED
    
    unsigned int ldr; //LDR value for room 
    unsigned short room_light_delay = 0;
    unsigned short status_old = ROOM_LIGHT_ON, status_current = ROOM_LIGHT_ON;
    int LED_on_count = 0;
    
    // Set High for Arduino !!!!!!!!!!!!!!!! CHANGE !!!!!!!!!!!!!!!!!!!!!!!
    LATCbits.LATC5 = 0b1; //interrupt high, failing low active on arduino
   
    
    while (1) {

        /********** Sleep *******************/

        WDTCONbits.SWDTEN = 0b1;

        SLEEP();
        LED_on_count++;
             
        WDTCONbits.SWDTEN = 0b0;
        
        /// Wake up from interrupt, switch was pressed
        if (switch_interrupt==1) {
                    
            switch_interrupt=0;
        
             // Phase one - dimm up the light
            if (light_on == 1) {

                ArduinoInterupt(ARDUINO_INTERUPT_SWITCH);
                status_old = ROOM_LIGHT_ON; // switch clicked, LED is on

                ldr=ReadLdr();
                
                if (ldr>DIMM_LIGHT_VALUE) {
                
                    Enable_PWM_LED_Dimmer_RA5(); // add

                    for (LED_on_count = 0; LED_on_count <700; LED_on_count+=1) { //10 bit  LED_on_counter
                        __delay_ms(1) ;
                        Set_PWM_RA5(LED_on_count);
                        if (light_on == 0) break;
                    }

                } else {
                    light_on =2;
                }
                
            }
            // Phase one - light is on
            if (light_on == 1) {
                for (int i = 0; i < 20; i++) {
                    if (light_on == 0) break;
                }
            }

            
            if (light_on != 2) {
            
                // Phase three - dimm down the light

                for (LED_on_count; LED_on_count > 0; LED_on_count-=4) {
                    Set_PWM_RA5(LED_on_count);
                }

                Disable_PWM_LED_Dimmer_RA5();
            }
            
            light_on=0;
        }
    
// ***************  Wakeup from Watchdog - Check if light in the room has changed
            
        if (STATUSbits.nTO==0) //Watchdog time out (page22)
        {

            ldr=ReadLdr();
        
            if (ldr > ROOM_LIGHT_VALUE)
                status_current = ROOM_LIGHT_OFF;
            else
                status_current = ROOM_LIGHT_ON;

            if (status_current != status_old) {

                room_light_delay++;

                if ((status_current == ROOM_LIGHT_ON)&&(room_light_delay > ROOM_LIGHT_COUNT_ON)) {
                    room_light_delay = 0;
                    status_old = status_current;
                    ArduinoInterupt(ARDUINO_INTERUPT_ROOM_LIGHT_ON);


                    WDTCONbits.WDTPS = CHECK_FREQUENCY_2S; // 

                }

                if ((status_current == ROOM_LIGHT_OFF)&&(room_light_delay > ROOM_LIGHT_COUNT_OFF)) {
                    room_light_delay = 0;
                    status_old = status_current;
                    ArduinoInterupt(ARDUINO_INTERUPT_ROOM_LIGHT_OFF);

                    WDTCONbits.WDTPS = CHECK_FREQUENCY_05S; // 

                }

            } else {
                room_light_delay = 0;
            }

        } // Switch interrupt
        
    }

}


void ArduinoInterupt(unsigned short time){
    
    LATCbits.LATC5 = 0b0;
    
    switch(time) {
        case ARDUINO_INTERUPT_ROOM_LIGHT_ON: __delay_us(ARDUINO_INTERUPT_ROOM_LIGHT_ON);break;
        case ARDUINO_INTERUPT_ROOM_LIGHT_OFF: __delay_us(ARDUINO_INTERUPT_ROOM_LIGHT_OFF);break;
        case ARDUINO_INTERUPT_SWITCH: __delay_us(ARDUINO_INTERUPT_SWITCH);break;
        default: break;
    }
    
      LATCbits.LATC5 = 0b1;
      
}


unsigned int ReadLdr() {

    unsigned int ldr;
    
    ANSELCbits.ANSC3 = 0b1; //PORTA ANALOG SELECT REGISTER

    ADCON0bits.ADON = 0b1; // turn on the ADC unit, consumes power, !!!!!!!!!!!!!!!!!!!!!!
    
    LATCbits.LATC4 = 0b1; //enable power for measurement

    ADCON0bits.GO_nDONE = 0b1; // start the conversion
    while (ADCON0bits.GO_nDONE); // wait for the conversion to finish
      
    ldr = (ADRESH << 8) | ADRESL; 

    LATCbits.LATC4 = 0b0; //disable power for measurement
    
    ADCON0bits.ADON = 0b0; // turn off the ADC unit, consumes power, !!!!!!!!!!!!!!!!!!!!!!
    
    return ldr;
}


void Enable_PWM_LED_Dimmer_RA5() {
/* http://www.micro-examples.com/public/microex-navig/doc/097-pwm-calculator.html
 * https://geekilyinteresting.wordpress.com/2013/09/06/playing-with-pics-4-dimming-leds-with-pwm/
 * 
 *  * PWM registers configuration
 * Fosc = 31000 Hz
 * Fpwm = 60.08 Hz (Requested : 60 Hz)
 * Duty Cycle = 50 %
 * Resolution is 9 bits
 * Prescaler is 1
 * Ensure that your PWM pin is configured as digital output
 * see more details on http://www.micro-examples.com/
 * this source code is provided 'as is',
 * use it at your own risks
 * 
 * PR2 = 0b10000000 ;
 * T2CON = 0b00000100 ;
 * CCPR1L = 0b01000000 ;
 * CCP1CON = 0b00011100 ;
 * 
 * 
 */   
    
// Disable the CCPx pin output driver by setting the associated TRIS bit.
TRISAbits.TRISA5 = 0b1; 


// ******* Define PWM Cycle Value *****************************

// Load the PRx register with the PWM period value.
PR2 = 0b10000000 ;
    
// Configure the CCP module for the PWM mode by loading the CCPxCON register with the appropriate values.
CCP2CON = 0b00011100 ; // 00=Single Output  01=two LSbs of the PWM duty cycle 1100=PWM mode: PxA, PxC active-high; PxB, PxD active-high

CCP2CONbits.CCP2M=0b1100; // PWM Mode
CCP2CONbits.P2M=00; //Single output; PxA modulated; PxB, PxC, PxD assigned as port pins

PSTR2CONbits.STR2A=0b1; // PxA pin has the PWM needed for EECP module

// *************** Define Cycle Times

//Load the CCPRxL register and the DCxBx bits of the CCPxCON register, with the PWM duty cycle value.
// TMRx = CCPRxH:CCPxCON<5:4>
CCPR1L = 0b00101000 ;
CCP2CONbits.DC2B=0b01 ;//  These bits are the two LSbs of the PWM duty cycle.      
        
// *************** Configure and start Timer2 ****************


// Select the Timer2/4/6 resource to be used for PWM generation by setting the
CCPTMRS0bits.C2TSEL=0b00; // Timer 2 used

//  Clear the TMR2IF interrupt flag bit of the PIR1 register.
PIR1bits.TMR2IF = 0;

//  Set the Timer2 prescale value by loading the T2CKPS bits of the T2CON register.
//  Enable Timer2 by setting the TMR2ON bit of the T2CON register.
T2CON = 0b00000100 ;

// Enable PWM output after a new PWM cycle has started:
// Wait until Timer2 overflows(TMR2IF bit of the PIR1 register is set).
// while(PIR1bits.TMR2IF == 0){}

// Enable the CCPx pin output driver by clearing the associated TRIS bit.
TRISAbits.TRISA5 = 0b0; 

return;
}




void Disable_PWM_LED_Dimmer_RA5() {

    PWM2CON = 0b00000000; // Clear PWMxCON
    T2CON=0;
    TRISAbits.TRISA5 = 0b1;    
//    CCPR2H = 0b0; //Clear pulse with register          
//    CCPR2L = 0b0; //

    return;
}

void Set_PWM_RA5(int duty) {
    // set the new duty cycle
    CCP2CONbits.DC2B = duty & 0b11;// LSB
    CCPR2L = duty >> 2;             // MSB
}