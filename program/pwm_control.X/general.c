/*-----------------------------------------------
 *
 * Last updated : 2020/02/01, 18:33
 * Author       : Takuto Jibiki
 *
-----------------------------------------------*/
#define FCY 40000000UL /* delay 用 (Fosc/2) */
#include <libpic30.h>  /* delay */
#include <stdbool.h>
#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "dsPIC33FJ128MC802.h"
#include "Controller_Protocol.h"
#include "general.h"
#include "config.h"

bool g_tx_flag = 1; /* 通信用 */

/*-----------------------------------------------
 *
 * 通信
 *
-----------------------------------------------*/
void EUSART_Write(unsigned char data)
{
    U1TXREG = data;
}

void EUSART_TxInterrupt_Control(bool enable_or_disable)
{
    U1TXIE = enable_or_disable;
}

bool EUSART_ERROR_from_master(void)
{
    if ((U1STAbits.FERR == 1) || (U1STAbits.OERR == 1))
    {
        /* フレーミングエラー処理 */
        uint8_t waste_data;
        waste_data = U1RXREG;
        /* オーバーランエラー処理 */
        U1MODE = 0x0808;
        U1STA = 0x2000;
        U1MODEbits.UARTEN = 1;
        U1STAbits.UTXEN = 1;
        return error;
    }
    return not_error;
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt()
{
    U1RXIF = 0;
    Store_Datas(Buffer0, U1RXREG, number_of_rxdata0 * 2 + 3,
                0, EUSART_ERROR_from_master);

    /* デバッグ */
    debug_LED1 ^= 1;
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt()
{
    g_tx_flag = Send_Till_EndSignal(TxData0,
                                    EUSART_Write,
                                    EUSART_TxInterrupt_Control,
                                    number_of_txdata0, 0);
    U1TXIF = disable;
}

/*-----------------------------------------------
 *
 * パラメータを読み込む
 *
-----------------------------------------------*/
void load_param(int16_t param[][PARAM_UNIT_NUM])
{
    size_t i = 0;
    size_t cnt = 0;
    while (1)
    {
        /* 受信 */
        Organize_Datas(RxData0, Buffer0, number_of_rxdata0, 0);

        /* 確認 */
        if (RxData0[0].all_data == i + 1)
            ++cnt;
        else if (RxData0[0].all_data == 255)
            break;
        else
            cnt = 0;

        /* 読み込む */
        if (cnt > 10)
        {
            param[i][0] = ASBL(RxData0[1].all_data, RxData0[2].all_data);
            param[i][1] = ASBL(RxData0[3].all_data, RxData0[4].all_data);
            ++i;
            cnt = 0;
        }

        /* 送信 */
        if (g_tx_flag)
        {
            TxData0[0] = i;
            Send_StartSignal(EUSART_Write, EUSART_TxInterrupt_Control, U1TXIE);
        }

        __delay_ms(5);
    }
}

/*-----------------------------------------------
 *
 * モータ関連
 *
-----------------------------------------------*/
/* モータを操作する */
void operate_motor(double pwm[])
{
    apply_pwm(pwm);
    apply_io(pwm);
}

/* PWM を反映する */
void apply_pwm(double pwm[])
{
    /* pwm の絶対値を 100 以下に収める */
    for (size_t i = 0; i < 2; ++i)
    {
        if (fabs(pwm[i]) > 100)
            pwm[i] = 100 * GET_SIGNAL_FLOAT(pwm[i]);
    }

    /* 反転して PWM を反映（MD が Active Low だから） */
    OC1RS = (uint16_t)(OC_PERIOD * (double)(100 - fabs(pwm[0])) * 1E-2);
    OC2RS = (uint16_t)(OC_PERIOD * (double)(100 - fabs(pwm[1])) * 1E-2);
}

/* IO を反映する */
void apply_io(double pwm[])
{
    if (pwm[0] == 0)
    {
        MOTOR_F1 = OFF;
        MOTOR_B1 = OFF;
    }
    else if (pwm[0] > 0)
    {
        MOTOR_F1 = ON;
        MOTOR_B1 = OFF;
    }
    else
    {
        MOTOR_F1 = OFF;
        MOTOR_B1 = ON;
    }

    if (pwm[1] == 0)
    {
        MOTOR_F2 = OFF;
        MOTOR_B2 = OFF;
    }
    else if (pwm[1] > 0)
    {
        MOTOR_F2 = ON;
        MOTOR_B2 = OFF;
    }
    else
    {
        MOTOR_F2 = OFF;
        MOTOR_B2 = ON;
    }
}