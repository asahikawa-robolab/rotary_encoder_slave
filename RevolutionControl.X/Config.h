#ifndef CONFIG_H
#define CONFIG_H

#include "Parameter.h"

/* Delay */
#define FCY 4000000UL                  /*delay関数用定義 Forc/2*/
#include <libpic30.h>                    /*delay関数用*/

// DSPIC33FJ128MC802 Configuration Bit Settings

// 'C' source line config statements

// DSPIC33FJ128MC802 Configuration Bit Settings

// 'C' source line config statements

/*-----------------------------------------------
 *
 * Config
 *
-----------------------------------------------*/
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


/*-----------------------------------------------
 *
 * Define
 *
-----------------------------------------------*/
/* ControllerProtocol */
#define bus1 0

/* debug */
#define debug_LED1 LATBbits.LATB2
#define ErrorLED LATBbits.LATB3

/* QEI */
#define QEI_INIT_VALUE 0x7FFF   /* 0x7FFF : 0xFFFF / 2 */

/*OC*/
#define OC_PERIOD 0x0800

/*PID*/
#define PID_TIMER_1MS 39201 /* 1 [ms] 分のカウント */
#define PID_PERIOD 50       /* 操作量を計算する周期 [ms] */

/* ゼロ点合わせ用のリミットスイッチが接続されている場所 */
#define Limit_1 PORTAbits.RA1
#define Limit_2 PORTAbits.RA0

/* 通信用 */
#define UP(data) ((((int16_t)data) >> 8) & 0xFF)
#define LOW(data) (((int16_t)data) & 0xFF)
#define ASBL(up, low) ((int16_t)((up) << 8) | (low)) /* Assemble */

/*-----------------------------------------------
 *
 * 構造体
 *
-----------------------------------------------*/
typedef struct
{
    uint16_t pre; /* previous */
    uint16_t cur; /* current */
} POSCNTS;

typedef struct
{
    double pwm;
    int c_rpm; /* currentRPM */
} MOTOR;

#endif