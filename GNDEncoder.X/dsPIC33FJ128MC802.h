#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H


#include <p33FJ128MC802.h>
/*********************************config*********************************/

// DSPIC33FJ128MC802 Configuration Bit Settings

// 'C' source line config statements

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Mode (Internal Fast RC (FRC))
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function (OSC2 pin has digital I/O function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = OFF               // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-low output polarity)
#pragma config HPOL = OFF               // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-low output polarity)
#pragma config PWMPIN = ON             // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PWM module at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

/*********************************config end*********************************/


//Timer settings
#define T1ON T1CONbits.TON
#define T1CKPS T1CONbits.TCKPS
#define T2ON T2CONbits.TON
#define T2CKPS T2CONbits.TCKPS
#define T3ON T3CONbits.TON
#define T3CKPS T3CONbits.TCKPS
#define T4ON T4CONbits.TON
#define T4CKPS T4CONbits.TCKPS
#define T5ON T5CONbits.TON
#define T5CKPS T5CONbits.TCKPS

//UART settings
#define T1ON T1CONbits.TON
#define T1CKPS T1CONbits.TCKPS
#define T2ON T2CONbits.TON
#define T2CKPS T2CONbits.TCKPS
#define T3ON T3CONbits.TON
#define T3CKPS T3CONbits.TCKPS
#define T4ON T4CONbits.TON
#define T4CKPS T4CONbits.TCKPS
#define U1RXISEL U1STAbits.URXISEL
#define UART1EN  U1MODEbits.UARTEN
#define U1TXEN   U1STAbits.UTXEN
#define U1OERR U1STAbits.OERR
#define U1FERR U1STAbits.FERR
#define U1RXDA   U1STAbits.URXDA
#define U2RXISEL U2STAbits.URXISEL
#define UART2EN  U2MODEbits.UARTEN
#define U2TXEN   U2STAbits.UTXEN
#define U2OERR U2STAbits.OERR
#define U2FERR U2STAbits.FERR
#define U2RXDA   U2STAbits.URXDA

//SPI setting
#define SPI1TBF SPI1STATbits.SPITBF
#define SPI1RBF SPI1STATbits.SPIRBF
#define SPI1ROV SPI1STATbits.SPIROV
#define SPI1EN SPI1STATbits.SPIEN
#define MSTEN_1 SPI1CON1bits.MSTEN
#define DISSDO_1 SPI1CON1bits.DISSDO
#define DISSCK_1 SPI1CON1bits.DISSCK
#define MODE16_1 SPI1CON1bits.MODE16
#define SMP_1 SPI1CON1bits.SMP
#define CKE_1 SPI1CON1bits.CKE
#define CKP_1 SPI1CON1bits.CKP
#define PPRE_1 SPI1CON1bits.PPRE
#define SPRE_1 SPI1CON1bits.SPRE
#define SSEN_1 SPI1CON1bits.SSEN
#define SPI2TBF SPI2STATbits.SPITBF
#define SPI2RBF SPI2STATbits.SPIRBF
#define SPI2ROV SPI2STATbits.SPIROV
#define SPI2EN SPI2STATbits.SPIEN
#define MSTEN_2 SPI2CON2bits.MSTEN
#define DISSDO_2 SPI2CON2bits.DISSDO
#define DISSCK_2 SPI2CON2bits.DISSCK
#define MODE26_2 SPI2CON2bits.MODE26
#define SMP_2 SPI2CON2bits.SMP
#define CKE_2 SPI2CON2bits.CKE
#define CKP_2 SPI2CON2bits.CKP
#define PPRE_2 SPI2CON2bits.PPRE
#define SPRE_2 SPI2CON2bits.SPRE
#define SSEN_2 SPI2CON2bits.SSEN

/*I2C setting*/
#define A10M_1 I2C1CONbits.A10M
#define ACKDT_1 I2C1CONbits.ACKDT
#define ACKEN_1 I2C1CONbits.ACKEN
#define DISSLW_1 I2C1CONbits.DISSLW
#define GCEN_1 I2C1CONbits.GCEN
#define I2CEN_1 I2C1CONbits.I2CEN
#define I2CSIDL_1 I2C1CONbits.I2CSIDL
#define IPMIEN_1 I2C1CONbits.IPMIEN
#define PEN_1 I2C1CONbits.PEN
#define RCEN_1 I2C1CONbits.RCEN
#define RSEN_1 I2C1CONbits.RSEN
#define SCLREL_1 I2C1CONbits.SCLREL
#define SEN_1 I2C1CONbits.SEN
#define SMEN_1 I2C1CONbits.SMEN
#define STREN_1 I2C1CONbits.STREN
#define ACKSTAT_1 I2C1STATbits.ACKSTAT
#define ADD10_1 I2C1STATbits.ADD10
#define BCL_1 I2C1STATbits.BCL
#define D_A_1 I2C1STATbits.D_A
#define GCSTAT_1 I2C1STATbits.GCSTAT
#define I2COV_1 I2C1STATbits.I2COV
#define IWCOL_1 I2C1STATbits.IWCOL
#define P_1 I2C1STATbits.P
#define RBF_1 I2C1STATbits.RBF
#define R_W_1 I2C1STATbits.R_W
#define S_1 I2C1STATbits.S
#define TBF_1 I2C1STATbits.TBF
#define TRSTAT_1 I2C1STATbits.TRSTAT

//QEI settings
#define UPDN1  QEI1CONbits.UPDN
#define UPDN2  QEI2CONbits.UPDN

/*Interrupt*/

#define U1TXIF IFS0bits.U1TXIF
#define U1RXIF IFS0bits.U1RXIF
#define U2TXIF IFS1bits.U2TXIF
#define U2RXIF IFS1bits.U2RXIF
#define SPI1IF IFS0bits.SPI1IF
#define SPI1EIF IFS0bits.SPI1EIF
#define SPI2IF IFS2bits.SPI2IF
#define SPI2EIF IFS2bits.SPI2EIF
#define INT0IF IFS0bits.INT0IF
#define QEI1IF IFS3bits.QEI1IF
#define QEI2IF IFS4bits.QEI2IF
#define T1IF IFS0bits.T1IF
#define T2IF IFS0bits.T2IF
#define T3IF IFS0bits.T3IF
#define T4IF IFS1bits.T4IF
#define T5IF IFS1bits.T5IF
#define U1TXIE IEC0bits.U1TXIE
#define U1RXIE IEC0bits.U1RXIE
#define U2TXIE IEC1bits.U2TXIE
#define U2RXIE IEC1bits.U2RXIE
#define SPI1IE IEC0bits.SPI1IE
#define SPI1EIE IEC0bits.SPI1EIE
#define SPI2IE IEC2bits.SPI2IE
#define SPI2EIE IEC2bits.SPI2EIE
#define INT0IE IEC0bits.INT0IE
#define QEI1IE IEC3bits.QEI1IE
#define QEI2IE IEC4bits.QEI2IE
#define T1IE IEC0bits.T1IE
#define T2IE IEC0bits.T2IE
#define T3IE IEC0bits.T3IE
#define T4IE IEC1bits.T4IE
#define T5IE IEC1bits.T5IE
#define INT0IP IPC0bits.INT0IP
#define U1TXIP IPC3bits.U1TXIP
#define U1RXIP IPC2bits.U1RXIP
#define U2TXIP IPC7bits.U2TXIP
#define U2RXIP IPC7bits.U2RXIP
#define SPI1IP IPC2bits.SPI1IP
#define SPI1EIP IPC2bits.SPI1EIP
#define SPI2IP IPC8bits.SPI2IP
#define SPI2EIP IPC8bits.SPI2EIP
#define QEI1IP IPC14bits.QEI1IP
#define QEI2IP IPC18bits.QEI2IP
#define T1IP IPC0bits.T1IP
#define T2IP IPC1bits.T2IP
#define T3IP IPC2bits.T3IP
#define T4IP IPC6bits.T4IP
#define T5IP IPC7bits.T5IP


//Peripheral Pin Select settings
#define RP0IN 0
#define RP1IN 1
#define RP2IN 2
#define RP3IN 3
#define RP4IN 4
#define RP5IN 5
#define RP6IN 6
#define RP7IN 7
#define RP8IN 8
#define RP9IN 9
#define RP10IN 10
#define RP11IN 11
#define RP12IN 12
#define RP13IN 13
#define RP14IN 14
#define RP15IN 15
#define RP0OUT RPOR0bits.RP0R
#define RP1OUT RPOR0bits.RP1R
#define RP2OUT RPOR1bits.RP2R
#define RP3OUT RPOR1bits.RP3R
#define RP4OUT RPOR2bits.RP4R
#define RP5OUT RPOR2bits.RP5R
#define RP6OUT RPOR3bits.RP6R
#define RP7OUT RPOR3bits.RP7R
#define RP8OUT RPOR4bits.RP8R
#define RP9OUT RPOR4bits.RP9R
#define RP10OUT RPOR5bits.RP10R
#define RP11OUT RPOR5bits.RP11R
#define RP12OUT RPOR6bits.RP12R
#define RP13OUT RPOR6bits.RP13R
#define RP14OUT RPOR7bits.RP14R
#define RP15OUT RPOR7bits.RP15R
/*Module*/
#define U1TXR 3
#define U2TXR 5
#define PWM1R 18
#define OPC1R 18
#define OPC2R 19
#define OPC3R 20
#define OPC4R 21
#define SDO1R 7
#define SCK1R_OUT 8
#define SSR 9
#define U1RXR RPINR18bits.U1RXR
#define U2RXR RPINR19bits.U2RXR
#define QEA1R RPINR14bits.QEA1R
#define QEB1R RPINR14bits.QEB1R
#define QEA2R RPINR16bits.QEA2R
#define QEB2R RPINR16bits.QEB2R
#define SDI1R RPINR20bits.SDI1R
#define SCK1R_IN RPINR20bits.SCK1R
#define SS1R RPINR21bits.SS1R

#endif	/* XC_HEADER_TEMPLATE_H */

