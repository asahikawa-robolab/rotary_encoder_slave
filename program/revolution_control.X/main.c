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
    PARAM_MAX_PWM,
    PARAM_ENCODER_POL,
    PARAM_ENCODER_RESOLUTION,
    PARAM_STOP_REV,
    PARAM_STOP_PWM,
    PARAM_KP
} ParamList;

/*-----------------------------------------------
 *
 * 宣言
 *
-----------------------------------------------*/
extern void Initialize(void);
void calc_encoder(double curr_rev[]);
void calc_rev(int64_t curr_cnt[], double curr_rev[]);
void calc_pwm(double pwm[], double curr_rev[]);
void check_pol(double pwm[]);
void inform_err(size_t ch);

/*-----------------------------------------------
 *
 * グローバル変数
 *
-----------------------------------------------*/
int64_t g_qei_int_cnt[2];                          /* QEI割り込みが発生した回数 */
int16_t g_target_rev[2];                           /* 目標回転数 */
int16_t g_param[MAX_NUM_OF_PARAM][PARAM_UNIT_NUM]; /* パラメータ */

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
    load_param(g_param);

    while (1)
    {
        /* 受信 */
        Organize_Datas(RxData0, Buffer0, number_of_rxdata0, 0);
        g_target_rev[0] = ASBL(RxData0[1].all_data, RxData0[2].all_data);
        g_target_rev[1] = ASBL(RxData0[3].all_data, RxData0[4].all_data);

        /* 動作周期調整 */
        __delay_ms(5);
    }
    return (EXIT_SUCCESS);
}

/*-----------------------------------------------
 *
 * タイマ割り込み
 *
-----------------------------------------------*/
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    static size_t TmrIntCnt; /* タイマー割り込みの発生した回数 */

    T1IF = CLEAR;
    PR1 = TIMER_1MS;
    ++TmrIntCnt;

    /* CALC_PERIOD [ms] 毎に真になる */
    if (TmrIntCnt >= CALC_PERIOD)
    {
        TmrIntCnt = CLEAR;

        double pwm[2];      /* 出力 PWM */
        double curr_rev[2]; /* 現在の回転数 */

        calc_encoder(curr_rev);  /* エンコーダに関する計算 */
        calc_pwm(pwm, curr_rev); /* pwm を計算 */
        operate_motor(pwm);      /* モータを操作 */
        check_pol(pwm);          /* エンコーダの極性の確認 */

        /* 送信 */
        if (g_tx_flag)
        {
            TxData0[0] = UP(curr_rev[0]);
            TxData0[1] = LOW(curr_rev[0]);
            TxData0[2] = UP(curr_rev[1]);
            TxData0[3] = LOW(curr_rev[1]);
            TxData0[4] = (int8_t)pwm[0];
            TxData0[5] = (int8_t)pwm[1];
            Send_StartSignal(EUSART_Write, EUSART_TxInterrupt_Control, U2TXIE);
        }
    }
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
 * エンコーダに関する計算
 *
-----------------------------------------------*/
void calc_encoder(double curr_rev[])
{
    /* 現在の位置カウントを計算 */
    int64_t curr_cnt[2];
    curr_cnt[0] = (int64_t)(65536 * g_qei_int_cnt[0] + POS1CNT);
    curr_cnt[1] = (int64_t)(65536 * g_qei_int_cnt[1] + POS2CNT);

    /* エンコーダの極性に合わせて反転 */
    for (size_t i = 0; i < 2; ++i)
    {
        if (g_param[PARAM_ENCODER_POL][i])
            curr_cnt[i] *= -1;
    }

    /* 回転数を計算 */
    calc_rev(curr_cnt, curr_rev);
}

/*-----------------------------------------------
 *
 * 回転数を計算
 *
-----------------------------------------------*/
void calc_rev(int64_t curr_cnt[], double curr_rev[])
{
    static int64_t pre_cnt[2]; /* 前回の位置カウント */

    /* 回転数を求めるときに使う係数を計算 */
    const double coef[2] = {
        1 / (g_param[PARAM_ENCODER_RESOLUTION][0] * 4.0) / (CALC_PERIOD * 1E-3) * 60.0,
        1 / (g_param[PARAM_ENCODER_RESOLUTION][1] * 4.0) / (CALC_PERIOD * 1E-3) * 60.0,
    };

    /* 回転数を計算 */
    for (size_t i = 0; i < 2; ++i)
        curr_rev[i] = (curr_cnt[i] - pre_cnt[i]) * coef[i];

    /* 前回の位置カウントを更新 */
    for (size_t i = 0; i < 2; ++i)
        pre_cnt[i] = curr_cnt[i];
}

/*-----------------------------------------------
 *
 * PWM を計算
 *
-----------------------------------------------*/
void calc_pwm(double pwm[], double curr_rev[])
{
    static double pre_pwm[2]; /* 前回の pwm */

    for (size_t i = 0; i < 2; ++i)
    {
        /* 比例制御 */
        pwm[i] = pre_pwm[i] + (g_target_rev[i] - curr_rev[i]) * g_param[PARAM_KP][i] * 1E-2;

        /* -100 ～ 100 に収める */
        pwm[i] = (fabs(pwm[i]) > 100)
                     ? 100 * GET_SIGNAL_FLOAT(pwm[i])
                     : pwm[i];

        /* pwm が大きくても curr_rev が小さい場合，
        駆動が OFF であるとみなして pwm の上限を stop_pwm に制限する */
        if (fabs(curr_rev[i]) < g_param[PARAM_STOP_REV][i] && fabs(pwm[i]) > g_param[PARAM_STOP_PWM][i])
            pwm[i] = g_param[PARAM_STOP_PWM][i] * GET_SIGNAL_FLOAT(pwm[i]);

        /* 回転数の偏差が閾値以下で，目標回転数がが０だったらモータを停止 */
        if (fabs(g_target_rev[i] - curr_rev[i]) < 10 && g_target_rev[i] == 0)
            pwm[i] = 0;

        /* kp が 0 のときは停止 */
        if (g_param[PARAM_KP][i] == 0)
            pwm[i] = 0;

        /* 前回の pwm を更新 */
        pre_pwm[i] = pwm[i];
    }
}

/*-----------------------------------------------
 *
 * エンコーダの極性を確認する
 *
-----------------------------------------------*/
void check_pol(double pwm[])
{
    static size_t err_cnt[2];

    for (size_t i = 0; i < 2; ++i)
    {
        if (fabs(pwm[i]) > g_param[PARAM_MAX_PWM][i] && g_param[PARAM_KP][i] != 0)
            ++err_cnt[i];
        else
            err_cnt[i] = CLEAR;

        if (err_cnt[i] > 3)
            inform_err(i);
    }
}

/*-----------------------------------------------
 *
 * LED を点滅させてエラーを知らせる
 *
-----------------------------------------------*/
void inform_err(size_t ch)
{
    /* モータを停止 */
    MOTOR_F1 = OFF;
    MOTOR_B1 = OFF;
    MOTOR_F2 = OFF;
    MOTOR_B2 = OFF;

    /* LED を光らせてエラーを知らせる */
    if (ch == 0)
    {
        while (1)
        {
            debug_LED1 = ON;
            __delay_ms(100);
            debug_LED1 = OFF;
            __delay_ms(100);
            debug_LED1 = ON;
            __delay_ms(100);
            debug_LED1 = OFF;
            __delay_ms(400);
        }
    }
    else
    {
        while (1)
        {
            debug_LED2 = ON;
            __delay_ms(100);
            debug_LED2 = OFF;
            __delay_ms(100);
            debug_LED2 = ON;
            __delay_ms(100);
            debug_LED2 = OFF;
            __delay_ms(400);
        }
    }
}