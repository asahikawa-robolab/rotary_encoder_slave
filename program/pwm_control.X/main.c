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
    PARAM_MAX_PWM
} ParamList;

/*-----------------------------------------------
 *
 * 宣言
 *
-----------------------------------------------*/
extern void Initialize();
void calc_pwm(double pwm[], int16_t param[][PARAM_UNIT_NUM]);

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

    /* モータ */
    double pwm[2] = {0, 0};

    while (1)
    {
        /* 受信 */
        Organize_Datas(RxData0, Buffer0, number_of_rxdata0, 0);
        pwm[0] = (int8_t)RxData0[1].all_data;
        pwm[1] = (int8_t)RxData0[2].all_data;

        /* PWM を計算 */
        calc_pwm(pwm, param);

        /* モータを操作 */
        operate_motor(pwm);

        /* 動作周期調整 */
        __delay_ms(5);
    }
    return (EXIT_SUCCESS);
}

/*-----------------------------------------------
 *
 * PWM を計算
 *
-----------------------------------------------*/
void calc_pwm(double pwm[], int16_t param[][PARAM_UNIT_NUM])
{
    for (size_t i = 0; i < 2; ++i)
    {
        /* 出力する PWM に上限を設定する */
        if (fabs(pwm[i]) > param[PARAM_MAX_PWM][i])
            pwm[i] = param[PARAM_MAX_PWM][i] * GET_SIGNAL_FLOAT(pwm[i]);

        /* PARAM_ENABLE が false だったらモータを駆動しない */
        if (param[PARAM_ENABLE][i] == false)
            pwm[i] = 0;
    }
}