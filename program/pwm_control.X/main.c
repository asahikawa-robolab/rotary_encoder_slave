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
    PARAM_ENABLE,
    PARAM_MAX_PWM,
    PARAN_MAX_PWM_CHANGE
} ParamList;

/*-----------------------------------------------
 *
 * 宣言
 *
-----------------------------------------------*/
extern void Initialize(void);
void calc_pwm(double pwm[]);

/*-----------------------------------------------
 *
 * グローバル変数
 *
-----------------------------------------------*/
int8_t g_target_pwm[2];                            /* 目標デューティー比 */
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
        g_target_pwm[0] = RxData0[1].all_data;
        g_target_pwm[1] = RxData0[2].all_data;

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

        double pwm[2];

        calc_pwm(pwm);      /* pwm を計算 */
        operate_motor(pwm); /* モータを操作 */
    }
}

/*-----------------------------------------------
 *
 * PWM を計算
 *
-----------------------------------------------*/
void calc_pwm(double pwm[])
{
    static double pre_pwm[2]; /* 前回のpwm */

    for (size_t i = 0; i < 2; i++)
    {
        /* 受信したpwmを代入 */
        pwm[i] = (int8_t)RxData0[i + 1].all_data;

        /* 変化量を計算 */
        double pwm_change = pwm[i] - pre_pwm[i];

        /* 変化量を PARAM_MAX_PWM_CHANGE に収める */
        if (fabs(pwm_change) > g_param[PARAN_MAX_PWM_CHANGE][i] / 20)
            pwm[i] = pre_pwm[i] + g_param[PARAN_MAX_PWM_CHANGE][i] / 20 * GET_SIGNAL_FLOAT(pwm_change);

        /* 更新 */
        pre_pwm[i] = pwm[i];
    }
}