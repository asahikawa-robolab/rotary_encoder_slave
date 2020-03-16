#define FCY 40000000UL /* delay 用 (Fosc/2) */
#include <libpic30.h>  /* delay */
#include <stdlib.h>
#include <xc.h>
#include <math.h>
#include "Controller_Protocol.h"
#include "dsPIC33FJ128MC802_config.h"
#include "dsPIC33FJ128MC802.h"
#include "general.h"
#include "config.h"
#include "setup.h"

/*-----------------------------------------------
 *
 * パラメータ
 *
-----------------------------------------------*/
typedef enum
{
    PARAM_ENCODER_POL,
    PARAM_ENCODER_RESOLUTION,
} ParamList;

/*-----------------------------------------------
 *
 * 宣言
 *
-----------------------------------------------*/
void calc_odometry(int32_t odometry[], int16_t param[][PARAM_UNIT_NUM]);

/*-----------------------------------------------
 *
 * グローバル変数
 *
-----------------------------------------------*/
int64_t g_qei_int_cnt[2]; /* QEI割り込みが発生した回数 */

/*-----------------------------------------------
 *
 * main
 *
-----------------------------------------------*/
int main(int argc, char **argv)
{
    /* 初期設定 */
    Initialize_Parameters(); /* 通信ライブラリ初期化 */
    setup_dspic();
    setup_pin();
    setup_peripheral_module();

    /* パラメータ読み込み */
    int16_t param[MAX_NUM_OF_PARAM][PARAM_UNIT_NUM];
    load_param(param);

    /* リセットフラグ */
    bool reset_flag[2] = {false, false};
    int32_t odometry[2] = {0, 0};

    while (1)
    {
        /* 受信 */
        Organize_Datas(RxData0, Buffer0, number_of_rxdata0, 0);
        reset_flag[0] = RxData0[1].all_data;
        reset_flag[1] = RxData0[2].all_data;

        /* リセット */
        if (reset_flag[0])
        {
            g_qei_int_cnt[0] = 0;
            POS1CNT = 0;
        }
        if (reset_flag[1])
        {
            g_qei_int_cnt[1] = 0;
            POS2CNT = 0;
        }

        /* オドメトリ計算 */
        calc_odometry(odometry, param);

        /* 送信 */
        if (g_tx_flag)
        {
            TxData0[0] = (int8_t)(odometry[0] & 0xFF);
            odometry[0] >>= 8;
            TxData0[1] = (int8_t)(odometry[0] & 0xFF);
            odometry[0] >>= 8;
            TxData0[2] = (int8_t)(odometry[0] & 0xFF);
            odometry[0] >>= 8;
            TxData0[3] = (int8_t)(odometry[0] & 0xFF);

            TxData0[4] = (int8_t)(odometry[1] & 0xFF);
            odometry[1] >>= 8;
            TxData0[5] = (int8_t)(odometry[1] & 0xFF);
            odometry[1] >>= 8;
            TxData0[6] = (int8_t)(odometry[1] & 0xFF);
            odometry[1] >>= 8;
            TxData0[7] = (int8_t)(odometry[1] & 0xFF);
            Send_StartSignal(EUSART_Write, EUSART_TxInterrupt_Control, U1TXIE);
        }

        /* 動作周期調整 */
        __delay_ms(5);
    }
    return (EXIT_SUCCESS);
}

/*-----------------------------------------------
 *
 * QEI 割り込み
 *
-----------------------------------------------*/
void __attribute__((interrupt, no_auto_psv)) _QEI1Interrupt()
{
    if (QEI1CONbits.UPDN)
        ++g_qei_int_cnt[0];
    else
        --g_qei_int_cnt[0];
    QEI1IF = CLEAR;
}

void __attribute__((interrupt, no_auto_psv)) _QEI2Interrupt()
{
    if (QEI2CONbits.UPDN)
        ++g_qei_int_cnt[1];
    else
        --g_qei_int_cnt[1];
    QEI2IF = CLEAR;
}

/*-----------------------------------------------
 *
 * オドメトリの計算
 *
-----------------------------------------------*/
void calc_odometry(int32_t odometry[], int16_t param[][PARAM_UNIT_NUM])
{
    /* 位置カウントを計算 */
    int64_t cnt[2];
    cnt[0] = (int64_t)(65536 * g_qei_int_cnt[0] + POS1CNT);
    cnt[1] = (int64_t)(65536 * g_qei_int_cnt[1] + POS2CNT);

    for (size_t i = 0; i < 2; ++i)
    {
        /* 位置カウントを角度に変換 */
        odometry[i] = (int32_t)(cnt[i] / (param[PARAM_ENCODER_RESOLUTION][i] * 4.0) * 360.0);

        /* PARAM_ENCODER_POL の値に応じて反転 */
        if (param[PARAM_ENCODER_POL][i] == true)
            odometry[i] *= -1;
    }
}