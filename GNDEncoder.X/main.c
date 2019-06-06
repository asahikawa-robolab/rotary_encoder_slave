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
/* Delay */
#define FCY 4000000UL                  /*delay関数用定義 Forc/2*/
#include <libpic30.h>                    /*delay関数用*/

/*-----------------------------------------------
 * Define
-----------------------------------------------*/
#define enable 1
#define disable 0
#define ON 1
#define OFF 0

/* ControllerProtocol */
#define bus1 0

/* for UART */
#define data_amount 150

/* debug */
#define debug_LED1 LATBbits.LATB2
#define debug_LED2 LATBbits.LATB3

/* QEI */
#define INIT_QEI_VALUE 0x7FFF   /* 0x7FFF : 0xFFFF / 2 */

/*-----------------------------------------------
 * 構造体
-----------------------------------------------*/
typedef struct _QEI
{
    bool reset;
    long count;
    short rotate, res;
}_QEI;

/*-----------------------------------------------
 * プロトタイプ宣言
-----------------------------------------------*/
void Initialize();
void __attribute__((interrupt, no_auto_psv)) _QEI1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void);
void EUSART_Write(unsigned char data);
void EUSART_TxInterrupt_Control(bool enable_or_disable);
short QEI_Sampling(unsigned char channel, _QEI QEI);

/*-----------------------------------------------
 * グローバル変数
-----------------------------------------------*/
/* ControllerProtocol */
bool TxFrag = 1;

/* for UART */
int status = 0;
char display[data_amount];
uint8_t MY_ADDRESS = 0x80;
bool rx_has_started = 0; //受信開始したフラグ

int main(int argc, char** argv)
{
    Initialize();   /* 初期化 */ 
    
    /*-----------------------------------------------
     * 変数
    -----------------------------------------------*/
    _QEI QEI[2];   /* エンコーダ関連 */
    QEI[0].count = 0;
    QEI[1].count = 0;
    
    while (1)
    {
        __delay_ms(30);     /* 通信の周期を調整するため */

        /* 受信 */
        if (status == reception_complete)
        {
            Reception_from_master_main();
            if (rx_has_started == 0)
                rx_has_started = 1;
            QEI[0].reset = RxData0[1].all_data;
            QEI[1].reset = RxData0[2].all_data;
        }

        /* 送信用にエンコーダのデータを計算 */
        QEI[0].count += QEI_Sampling(0, QEI[0]);
        QEI[0].rotate = QEI[0].count / 1200;
        QEI[0].res = QEI[0].count % 1200;
        QEI[1].count += QEI_Sampling(1, QEI[1]);
        QEI[1].rotate = QEI[1].count / 1200;
        QEI[1].res = QEI[1].count % 1200;
        
        /* 送信 */
        if (TxFrag)
        {   
            TxData0[0] = QEI[0].rotate >> 8;
            TxData0[1] = QEI[0].rotate & 0xFF;
            TxData0[2] = QEI[0].res >> 8;
            TxData0[3] = QEI[0].res & 0xFF;
            TxData0[4] = QEI[1].rotate >> 8;
            TxData0[5] = QEI[1].rotate & 0xFF;
            TxData0[6] = QEI[1].res >> 8;
            TxData0[7] = QEI[1].res & 0xFF;    
            Send_StartSignal(EUSART_Write, EUSART_TxInterrupt_Control, U2TXIE);
        }
        
        if(QEI1IF)
            debug_LED1 = true;
    }
    return (EXIT_SUCCESS);
}

/*-----------------------------------------------
 * 一回前に呼び出されたときのカウント pre_POSxCNT と POSxCNT の差分を返す
-----------------------------------------------*/
short QEI_Sampling(unsigned char channel, _QEI QEI)
{
    short result, POSxCNT;
    static unsigned short pre_POSxCNT[2] = { INIT_QEI_VALUE, INIT_QEI_VALUE };
    bool QEIxIF;
    
    /* POSxCNT, QEIxIF をセット */
    if(channel == 0)
    {
        POSxCNT = POS1CNT;
        QEIxIF = QEI1IF;
    }
    else if(channel == 1)
    {
        POSxCNT = POS2CNT;
        QEIxIF = QEI2IF;
    }
    else while(1);

    /* QEI のリセットフラグが有効のとき */
    if(QEI.reset)
    {
        pre_POSxCNT[channel] = INIT_QEI_VALUE;
        return 0;
    }
    
    /* QEI の割り込みが有効，つまり POSxCNT レジスタがオーバーフローもしくはアンダーフローした場合 */
    if (QEIxIF)
    {
        /* アンダーフロー */
        if (POSxCNT > pre_POSxCNT[channel])
            result = POSxCNT + 65536 - pre_POSxCNT[channel];
        /* オーバーフロー */
        else if (POSxCNT < pre_POSxCNT[channel])
            result = POSxCNT - 65536 - pre_POSxCNT[channel];
        
        /* 割り込みフラグクリア */
        if(channel == 0) QEI1IF = false;
        else if(channel == 1) QEI2IF = false;
    }
    /* オーバーフローもアンダーフローも起こらなかったとき */
    else
        result = POSxCNT - pre_POSxCNT[channel];

    /* 値を更新 */
    pre_POSxCNT[channel] = POSxCNT;

    return result;
}

void EUSART_Write(unsigned char data) {
    U2TXREG = data;
}

void EUSART_TxInterrupt_Control(bool enable_or_disable) {
    U2TXIE = enable_or_disable;
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt()
{
    U1RXIF = 0;
    status = Reception_from_master(MY_ADDRESS);
}

void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt() {
    debug_LED2 ^= 1;
    TxFrag = Send_Till_EndSignal(TxData0, EUSART_Write, EUSART_TxInterrupt_Control, number_of_txdata0, bus1);
    U2TXIF = disable;
}

void Initialize() {
    int i;
    Initialize_Parameters();
    /*Oscillator*/
    OSCTUN = 0;
    PLLFBD = 16; //M=18
    CLKDIVbits.PLLPRE = 0; //PLL prescaler : /2  = N1
    CLKDIVbits.PLLPOST = 0; //PLL postscaler : /2 = N2
    //#pragma config FNOSC = LPRCDIVN より　Fin=7.37MHz(FRC)
    /* ↑ より　Fosc=Fin*(M/(N1*N2)) =33.165MHz
     Fcy=16.5825MHz*/

    /*Interrupt*/
#if 1 //割り込みする:1 しない:0
    SRbits.IPL = 0; // CPU interrupt priority level:zero
    CORCONbits.IPL3 = 0; //CPU interrupt priority level:<7
    INTCON1bits.NSTDIS = 0; //Allow interrupt nesting
    INTCON2bits.ALTIVT = 0; //use primary vector table

    //IEC0bits.OC1IE = enable;
    U1TXIE = disable; //UART1 tx interrupt enable
    U2TXIE = disable;
    U1RXIE = enable;
    // IEC3bits.QEI1IE = disable;
    T1IE = disable; //timer1 interrupt enable
    //IEC0bits.T2IE = disable;
    //IPC3bits.U1TXIP = 0x7; //interrupt is priority 6
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
    PMD1bits.U1MD = disable; //U1 disable:disable
    // U1MODE = 0x8808;
    U1MODEbits.UARTEN = enable; //UART1 is enabled
    U1MODEbits.USIDL = 0; //continue module operation in Idle mode
    U1MODEbits.IREN = disable; //IrDA disable
    U1MODEbits.RTSMD = 1; //U1RTS pin in simplex mode-単信
    U1MODEbits.UEN = 0x00; //TX,RX are enabled and used, CTS and RTS are controlled by port latches
    U1MODEbits.LPBACK = disable; //disable loopback
    U1MODEbits.ABAUD = disable; //auto baud rate disa

    U1MODEbits.URXINV = 0; //receive idle status: 1 ->use pull-up resistor on the rx pin
    /*UART1 is enabled, continue module operation in idle mode,IrDA disable, U1RTS pin in simplex mode, 
     * TX&RX pin are enabled and used, disable loopback, auto baud rate disable, high baud rate enable,receive idle status:1*/
    U1BRG = 15;

    /*UART2*/
    PMD1bits.U2MD = disable; //U1 disable:disable
    U2MODEbits.UARTEN = enable; //UART1 is enabled
    U2MODEbits.USIDL = 0; //continue module operation in Idle mode
    U2MODEbits.IREN = disable; //IrDA disable
    U2MODEbits.RTSMD = 1; //U1RTS pin in simplex mode-単信
    U2MODEbits.UEN = 0x00; //TX,RX are enabled and used, CTS and RTS are controlled by port latches
    U2MODEbits.LPBACK = disable; //disable loopback
    U2MODEbits.ABAUD = disable; //auto baud rate disable
    U2MODEbits.BRGH = 1; //high baud rate enable
    U2MODEbits.URXINV = 0; //receive idle status: 0 ->use pull-down resistor on the rx pin
    U2BRG = 15;

    /*transmission*/
    //#1
    U1STAbits.UTXISEL1 = 1; //transmit interrupt :when transmit buffer becomes empty
    U1STAbits.UTXEN = enable; //transmit enable
    U1TXIF = disable; //Interrupt flag disable

    U2STAbits.UTXISEL1 = 1; //transmit interrupt :when transmit buffer becomes empty
    U2STAbits.UTXEN = enable; //transmit enable
    U2TXIF = disable; //Interrupt flag disable

    /*reception*/
    //#2
    U1STAbits.URXISEL = 0x00; //receive interrupt set: transfer making the receive buffer full
    U1STAbits.ADDEN = disable; //address detect disable 
    U1RXIF = disable; //Interrupt flag disable

    /*QEI1*/
    QEI1CONbits.QEISIDL = disable; //continue module operation in idle mode
    QEI1CONbits.QEIM = 0x7; //QEI enabled (x4 mode) with position counter reset by match(MAX1CNT)
    QEI1CONbits.SWPAB = disable; //A and B inputs not swapped-交換
    QEI1CONbits.PCDOUT = disable; //direction status output disabled
    QEI1CONbits.TQGATE = disable;
    QEI1CONbits.UPDN = 1; //position counter direction is Positive
    MAX1CNT = 0xFFFF;
    POS1CNT = INIT_QEI_VALUE;   /* カウントリセット */
    QEI1IF = false;

    /*QEI2*/
    QEI2CONbits.QEISIDL = disable; //continue module operation in idle mode
    QEI2CONbits.QEIM = 0x7; //QEI enabled (x4 mode) with position counter reset by match(MAX1CNT)
    QEI2CONbits.SWPAB = disable; //A and B inputs not swapped-交換
    QEI2CONbits.PCDOUT = disable; //direction status output disabled
    QEI2CONbits.TQGATE = disable;
    QEI2CONbits.UPDN = 1; //position counter direction is Positive
    MAX2CNT = 0xFFFF;
    POS2CNT = INIT_QEI_VALUE;   /* カウントリセット */
    QEI2IF = false;

    /*Timer1*/
//    T1CONbits.TCS = T1CONbits.TGATE = T1CONbits.TSYNC = 0; //Timer mode
//    T1CONbits.TON = OFF; //Timer off
//    T1CONbits.TSIDL = 0; //Continue module operation in Idle mode
//    T1CONbits.TCKPS = 0x01; //prescale 1:8
//    PR1 = 22727; //50ms   1818; //4ms  
//    T1CONbits.TON = ON; //Timer on   

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

    for (i = 0; i < data_amount; i++)
        display[i] = 0;
}