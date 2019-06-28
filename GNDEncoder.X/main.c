/*
 * File:   main.c
 * Author: pguz1
 *
 * Created on 2019/01/02, 22:38
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <p33FJ128MC802.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Controller_Protocol.h"
#include "communication.h"
#include "variable.h"
#include "dsPIC33FJ128MC802.h"
#include "Parameter.h"
/* Delay */
#define FCY 4000000UL   /* delay関数用定義 Forc/2 */
#include <libpic30.h>   /* delay関数用 */

/*-----------------------------------------------
 *
 * 基板毎の設定
 *
-----------------------------------------------*/
uint8_t MY_ADDRESS = 0x00; /* この基板のアドレス */


/*-----------------------------------------------
 * Define
-----------------------------------------------*/
/* ControllerProtocol */
#define bus1 0

/* debug */
#define debug_LED1 LATBbits.LATB2
#define ErrorLED LATBbits.LATB3

/* 通信用 */
#define UP(data) ((((int16_t)data) >> 8) & 0xFF)
#define LOW(data) (((int16_t)data) & 0xFF)
#define ASBL(up, low) ((int16_t)((up) << 8) | (low)) /* Assemble */

/*-----------------------------------------------
 * プロトタイプ宣言
-----------------------------------------------*/
void Initialize();
void CalcOdometry(short Rev[], short Residue[], bool Reset[]);
void EUSART_Write(unsigned char data);
void EUSART_TxInterrupt_Control(bool enable_or_disable);

/*-----------------------------------------------
 * グローバル変数
-----------------------------------------------*/
/* 通信フラグ */
bool TxFrag = 1;
bool rx_has_started = 0; /* 受信開始したフラブ */
int status = 0;

short IntCnt[2] = {0, 0}; /* 割り込み回数 */
uint8_t Mode;

int main(int argc, char** argv)
{
    Initialize(); /* 初期化 */

    short Rev[2] = {0, 0}; /* 回転 */
    short Residue[2] = {0, 0}; /* １回転に満たないカウント */
    bool Reset[2] = {false, false}; /* カウントクリアフラグ */

    POS1CNT = POS2CNT = 0;

    while (1)
    {
        /*-----------------------------------------------
         * 受信
        -----------------------------------------------*/
        if (status == reception_complete)
        {
            Reception_from_master_main();
            if (rx_has_started == 0)
            {
                rx_has_started = 1;
            }
            Reset[0] = RxData0[1].all_data;
            Reset[1] = RxData0[2].all_data;
            Mode = RxData0[3].all_data;
        }

        /*-----------------------------------------------
         * オドメトリを計算
        -----------------------------------------------*/
        CalcOdometry(Rev, Residue, Reset);

        /*-----------------------------------------------
         * 送信
        -----------------------------------------------*/
        if (TxFrag)
        {
            TxData0[0] = UP(Rev[0]);
            TxData0[1] = LOW(Rev[0]);
            TxData0[2] = UP(Residue[0]);
            TxData0[3] = LOW(Residue[0]);
            TxData0[4] = UP(Rev[1]);
            TxData0[5] = LOW(Rev[1]);
            TxData0[6] = UP(Residue[1]);
            TxData0[7] = LOW(Residue[1]);

            __delay_ms(10 * number_of_txdata0);

            Send_StartSignal(EUSART_Write, EUSART_TxInterrupt_Control, U2TXIE);
        }
    }
    return (EXIT_SUCCESS);
}

void CalcOdometry(short Rev[], short Residue[], bool Reset[])
{
    /*-----------------------------------------------
     * リセット
    -----------------------------------------------*/
    if (Reset[0] == true)
    {
        IntCnt[0] = 0;
        POS1CNT = 0;
    }
    if (Reset[1] == true)
    {
        IntCnt[1] = 0;
        POS2CNT = 0;
    }
    ErrorLED = Reset[0] | Reset[1];

    /*-----------------------------------------------
     * POSxCNT と IntCnt から Rev と Residue 計算する
    -----------------------------------------------*/
    int32_t PosCnt[2] = {0, 0}; /* 割り込みと符号を考慮した QEI のカウント */
    short Coef = EncoderResolution[Mode] * 4; /* 回転を求めるときに使用する係数（coefficient） */
    /* X */
    PosCnt[0] = IntCnt[0] * 65536 + POS1CNT;
    Rev[0] = PosCnt[0] / Coef;
    Residue[0] = (PosCnt[0] % Coef) * 360 / Coef; /* [cnt] → [deg] */
    if (PolInverse[Mode][0] == true)
    {
        Rev[0] *= -1;
        Residue[0] *= -1;
    }
    /* Y */
    PosCnt[1] = IntCnt[1] * 65536 + POS2CNT;
    Rev[1] = PosCnt[1] / Coef;
    Residue[1] = (PosCnt[1] % Coef) * 360 / Coef; /* [cnt] → [deg] */
    if (PolInverse[Mode][1] == true)
    {
        Rev[1] *= -1;
        Residue[1] *= -1;
    }
}

/* QEI 割り込み */
void __attribute__((interrupt, no_auto_psv)) _QEI1Interrupt()
{
    if (QEI1CONbits.UPDN == true)
    {
        IntCnt[0]++;
    }
    else
    {
        IntCnt[0]--;
    }
    QEI1IF = false;
}

void __attribute__((interrupt, no_auto_psv)) _QEI2Interrupt()
{
    if (QEI2CONbits.UPDN == true)
    {
        IntCnt[1]++;
    }
    else
    {
        IntCnt[1]--;
    }
    QEI2IF = false;
}

void EUSART_Write(unsigned char data)
{
    U2TXREG = data;
}

void EUSART_TxInterrupt_Control(bool enable_or_disable)
{
    U2TXIE = enable_or_disable;
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt()
{
    U1RXIF = 0;
    status = Reception_from_master(MY_ADDRESS);
}

void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt()
{
    debug_LED1 ^= 1;
    TxFrag = Send_Till_EndSignal(TxData0, EUSART_Write, EUSART_TxInterrupt_Control, number_of_txdata0, bus1);
    U2TXIF = disable;
}

void Initialize()
{
    Initialize_Parameters();
    /*Oscillator*/
    OSCTUN = 18;
    PLLFBD = 40 - 2; //M=18
    CLKDIVbits.PLLPRE = 0; //PLL prescaler : /2  = N1
    CLKDIVbits.PLLPOST = 0; //PLL postscaler : /2 = N2
    //#pragma config FNOSC = FRC より Fin=7.37MHz
    /* ↑ より Fosc=Fin*(M/(N1*N2)) =78.4MHz≒80MHz
     Fcy≒40MHz=80MHz/2*/

    /*Initiate Clock Switch to Fast RC oscillator with PLL(NOSC=0b001)*/
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(0x01);

    /*Wait for Clock switch to occur
     * (current oscillator が0b001＝FRCでない間は待機)*/
    while (OSCCONbits.COSC != 0b001);

    /*Wait for PLL to lock*/
    while (OSCCONbits.LOCK != 1);

    /*Interrupt*/
#if 1 //割り込みする:1 しない:0
    SRbits.IPL = 0; // CPU interrupt priority level:zero
    CORCONbits.IPL3 = 0; //CPU interrupt priority level:<7
    INTCON1bits.NSTDIS = 0; //Allow interrupt nesting
    INTCON2bits.ALTIVT = 0; //use primary vector table

    U1TXIE = disable; //UART1 tx interrupt enable
    U2TXIE = disable;
    U1RXIE = enable;
    T1IE = disable; //timer1 interrupt enable
    QEI1IE = enable; /* QEI 割り込み */
    QEI2IE = enable;
    U2TXIP = 0x7; //Interrupt priority level:6
    U1RXIP = 0x7; //Interrupt priority level:6
    QEI1IP = 0x5;
    T1IP = 0x0; //set interrupt priority level
#endif
    /*pin*/
    TRISBbits.TRISB2 = 0; //debug1
    TRISBbits.TRISB3 = 0; //debug2

    TRISBbits.TRISB15 = 0; //UART2 transmission
    RPOR7bits.RP15R = 0b00101; //RP15-RB15: UART2 transmission

    TRISBbits.TRISB11 = 1; //reception
    U1RXR = 0xB; //RP11 - UART Reception

    TRISBbits.TRISB7 = 1; //QEI1A
    QEA1R = 0x7; //RP11
    TRISBbits.TRISB8 = 1; //QEI1B
    QEB1R = 0x8; //RP10

    TRISBbits.TRISB10 = 1; //QEI2
    QEA2R = 0xA;
    TRISBbits.TRISB9 = 1;
    QEB2R = 0x9;

    TRISBbits.TRISB12 = 0; //motor1-front
    TRISAbits.TRISA2 = 0; //motor1-back

    TRISAbits.TRISA3 = 0; //motor2-front
    TRISBbits.TRISB13 = 0; //motor2-back

    TRISBbits.TRISB14 = 0; //OC1 -RB14
    RPOR7bits.RP14R = 0b10010; //OC1 -RB14

    TRISBbits.TRISB4 = 0; //OC2-RB4
    RPOR2bits.RP4R = 0b10011;

    AD1PCFGL = 0xFFFF; //つけたら動かなかった・・・


    /*UART1*/
    //PMD1bits.U1MD = disable; //U1 disable:disable
    U1MODE = 0x0808;
    U2STA = 0x2000;
    U1TXIF = disable; //Interrupt flag disable
    U1RXIF = disable; //Interrupt flag disable

    /*UART1 is enabled, continue module operation in idle mode,IrDA disable, U1RTS pin in simplex mode,
     * TX&RX pin are enabled and used, disable loopback, auto baud rate disable, high baud rate enable,receive idle status:1*/
    U1BRG = 165;
    U1MODEbits.UARTEN = enable; //UART1 is enabled
    U1STAbits.UTXEN = enable; //transmit enable

    /*UART2*/
    //PMD1bits.U2MD = disable; //U1 disable:disable
    U2MODE = 0x0808;
    U2STA = 0x2000;
    U2TXIF = disable; //Interrupt flag disable
    U2BRG = 165;
    U2MODEbits.UARTEN = enable; //UART1 is enabled
    U2STAbits.UTXEN = enable; //transmit enable

    /*QEI1*/
    QEI1CONbits.QEISIDL = disable; //continue module operation in idle mode
    QEI1CONbits.QEIM = 0x7; //QEI enabled (x4 mode) with position counter reset by match(MAX1CNT)
    QEI1CONbits.SWPAB = disable; //A and B inputs not swapped-交換
    QEI1CONbits.PCDOUT = disable; //direction status output disabled
    QEI1CONbits.TQGATE = disable;
    QEI1CONbits.UPDN = 1; //position counter direction is Positive
    MAX1CNT = 0xFFFF;
    POS1CNT = 0; /* カウントリセット */
    QEI1IF = false;

    /*QEI2*/
    QEI2CONbits.QEISIDL = disable; //continue module operation in idle mode
    QEI2CONbits.QEIM = 0x7; //QEI enabled (x4 mode) with position counter reset by match(MAX1CNT)
    QEI2CONbits.SWPAB = disable; //A and B inputs not swapped-交換
    QEI2CONbits.PCDOUT = disable; //direction status output disabled
    QEI2CONbits.TQGATE = disable;
    QEI2CONbits.UPDN = 1; //position counter direction is Positive
    MAX2CNT = 0xFFFF;
    POS2CNT = 0; /* カウントリセット */
    QEI2IF = false;

    /*Timer2(for OC1)*/
    T2CONbits.TCS = T2CONbits.TGATE = T2CONbits.T32 = OFF; //Mode:16bit Timer
    T2CONbits.TON = ON;

    /*OC1*/
    OC1CONbits.OCM = 0x6; //Mode of OC1:PWM Mode.Fault pin disabled
    OC1CONbits.OCSIDL = 1; //halts in CPU idle mode
    OC1CONbits.OCTSEL = 0; //timer2
    // OC1RS = 2220; //decide the length of HI:0.5ms
    PR2 = 65535; //decide the length of period:144ms

    /*Timer3(for OC1)*/
    //    T3CONbits.TCS = T3CONbits.TGATE = OFF; //Mode:16bit Timer
    //    T3CONbits.TON = OFF;

    /*OC2*/
    OC2CONbits.OCM = 0x6; //Mode of OC2:PWM Mode.Fault pin disabled
    OC2CONbits.OCSIDL = 1; //halts in CPU idle mode
    OC2CONbits.OCTSEL = 0; //timer2
    // OC1RS = 2220; //decide the length of HI:0.5ms
    PR2 = 65535; //decide the length of period:144ms
}