/*-----------------------------------------------
 *
 * Last updated : 2020/02/05, 15:54
 * Author       : Takuto Jibiki
 *
-----------------------------------------------*/
#include <stdbool.h>
#include <xc.h>
#include "general.h"
#include "config.h"
#include "dsPIC33FJ128MC802.h"

void setup_dspic(void)
{
    /* Fosc = Fin * (M / (N1 * N2)) = 78.4 MHz ≒ 80 MHz */
    /* Fcy ≒ 40 MHz */
    OSCTUN = 18;            /* Fin = 7.84 MHz */
    PLLFBD = 40 - 2;        /* M = 40 */
    CLKDIVbits.PLLPRE = 0;  /* N1 = 2 */
    CLKDIVbits.PLLPOST = 0; /* N2 = 2 */

    CLKDIVbits.FRCDIV = 0;           /* 内部クロックプリスケーラ：1 */
    RCONbits.SWDTEN = 0;             /* WDT 許可しない */
    __builtin_write_OSCCONH(0x01);   /* 内部クロック + PLL */
    __builtin_write_OSCCONL(0x01);   /* 第２ｘオシレータ許可 */
    while (OSCCONbits.COSC != 0b001) /* 第１オシレータが「内部クロック+PLL」になるまで待機 */
        ;
    while (OSCCONbits.LOCK != 1) /* PLL安定化待ち */
        ;
}

void setup_pin(void)
{
    /* PIN */
    AD1PCFGL = 0xFFFF; /* ポートピンをデジタルモードに */
    TRISB2 = 0;        /* debug_LED1 */
    TRISB3 = 0;        /* debug_LED2 */
    TRISB15 = 0;       /* UART1 TX */
    TRISB11 = 1;       /* UART1 RX */
    TRISB7 = 1;        /* QEI1A */
    TRISB8 = 1;        /* QEI1B */
    TRISB10 = 1;       /* QEI2 */
    TRISB9 = 1;        /* QEI2 */
    // TRISB12 = 0;       /* MOTOR_F1 */
    // TRISA2 = 0;        /* MOTOR_B1 */
    // TRISA3 = 0;        /* MOTOR_F2 */
    // TRISB13 = 0;       /* MOTOR_B2 */
    // TRISB14 = 0;       /* OC1 */
    // TRISB4 = 0;        /* OC2 */

    /* PPS */
    RP15OUT = U1TXR; /* UART1 TX - RB15 */
    U1RXR = RP11IN;  /* UART1 RX - RB11 */
    // RP14OUT = OC1R_; /* OC1 - RB14 */
    // RP4OUT = OC2R_;  /* OC2 - RB4 */
    QEA1R = RP7IN;   /* QEA1 - RB7 */
    QEB1R = RP8IN;   /* QEB1 - RB8 */
    QEA2R = RP10IN;  /* QEA2 - RB10 */
    QEB2R = RP9IN;   /* QEB2 - RB9 */
}

void setup_peripheral_module(void)
{
    /* UART1 */
    U1RXIE = enable; /* 受信割り込み許可 */
    U1MODE = 0x0808;
    U1STA = 0x2000;
    U1BRG = 165;
    U1TXIF = disable; /* 割り込みフラグクリア */
    U1RXIF = disable; /* 割り込みフラグクリア */
    UART1EN = enable; /* UART1 有効化 */
    U1TXEN = enable;  /* 送信トランスミッタ有効 */

    /* UART2 */
    // U2MODE = 0x0808;
    // U2STA = 0x2000;
    // U2BRG = 165;
    // U2TXIF = disable; /* 割り込みフラグクリア */
    // U2RXIF = disable; /* 割り込みフラグクリア */
    // UART2EN = enable; /* UART2 有効化 */
    // U2TXEN = enable;  /* 送信トランスミッタ有効 */

    /* QEI1 */
    QEI1IE = enable;             /* QEI1 割り込み許可 */
    QEI1CONbits.QEIM = 0b111;    /* QEI 有効化，x4 モード */
    QEI1CONbits.SWPAB = disable; /* A 相と B 相のスワップを無効 */
    MAX1CNT = 0xFFFF;            /* 最大カウントレジスタ */
    POS1CNT = 0;                 /* 位置カウントリセット */

    /* QEI2 */
    QEI2IE = enable;             /* QEI2 割り込み許可 */
    QEI2CONbits.QEIM = 0b111;    /* QEI 有効化，x4 モード */
    QEI2CONbits.SWPAB = disable; /* A 相と B 相のスワップを無効 */
    MAX2CNT = 0xFFFF;            /* 最大カウントレジスタ */
    POS2CNT = 0;                 /* 位置カウントリセット */

    /* Timer1 */
    // T1IE = enable; /* タイマ割り込み許可 */
    // T1CKPS = 0b00; /* プリスケール値１：１ */
    // TMR1 = 0;
    // PR1 = TIMER_1MS;
    // T1ON = ON; /* Timer ON ビット */

    /* OC 用（先に Timer の設定をしなければならない） */
    /* Timer2 */
    // T1CKPS = 0b00;   /* プリスケール値１：１ */
    // PR2 = OC_PERIOD; /* 周期を決定 */
    // T2ON = ON;       /* Timer ON ビット */

    /* OC1 */
    // OC1M = 0b110; /* フォルト保護なしの PWM モード */
    // OC1TSEL = 0;  /* クロック源を Timer2 に */

    /* OC2 */
    // OC2M = 0b110; /* フォルト保護なしの PWM モード */
    // OC2TSEL = 0;  /* クロック源を Timer2 に */

    /* ピン変化割り込み */
    // CN2IE = enable; /* 割り込みを発生させるピンを選択 */
    // CN3IE = enable;
    // CNIF = 0;      /* 割り込みフラグクリア */
    // CNIE = enable; /* 割り込み許可 */
}