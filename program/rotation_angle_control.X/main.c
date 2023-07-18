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
    PARAM_ZERO_POINT_PWM,
    PARAM_MAX_PWM,
    PARAM_MIN_PWM,
    PARAM_STOP_REV,
    PARAM_STOP_PWM,
    PARAM_PWM_CHANGE,
    PARAM_KP,
    PARAM_KD,
    PARAM_KI,
    PARAM_PERMISSIBLE_ERR,
    PARAM_ENCODER_POL,
    PARAM_ENCODER_RESOLUTION,
    PARAM_ERROR_STOP
} ParamList;

/*-----------------------------------------------
 *
 * 宣言
 *
-----------------------------------------------*/
void zero_point_pwm(double pwm[], size_t ch, double sign);
void calc_encoder(int64_t curr_cnt[], double curr_rev[], size_t ch);
void calc_rev(int64_t curr_cnt[], double curr_rev[], size_t ch);
void calc_pwm(int64_t curr_cnt[], int16_t angle_diff[], double curr_rev[], double pwm[], size_t ch);
int64_t calc_cnt_diff(int64_t curr_cnt[], int16_t angle_diff[], size_t ch);
void smoothen_pwm_change(double pwm[], size_t ch);
void check_pol(int16_t angle_diff[], double pwm[], size_t ch);
void inform_err(size_t ch);

/*-----------------------------------------------
 *
 * グローバル変数
 *
-----------------------------------------------*/
bool g_zero_point_done[2];                         /* ゼロ点合わせの状態 */
int64_t g_qei_int_cnt[2];                          /* QEI 割り込みが発生した回数 */
int16_t g_target_angle[2];                         /* 目標回転角 */
int16_t g_param[MAX_NUM_OF_PARAM][PARAM_UNIT_NUM]; /* パラメータ */
int ms_count = 0;

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
        g_target_angle[0] = ASBL(RxData0[1].all_data, RxData0[2].all_data);
        g_target_angle[1] = ASBL(RxData0[3].all_data, RxData0[4].all_data);

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

        double pwm[2] = {0, 0}; /* 出力 PWM */
        double curr_rev[2];     /* 現在の回転数 */
        int64_t curr_cnt[2];    /* 現在の位置カウント */
        int16_t angle_diff[2];  /* 回転角偏差 */

        for (size_t i = 0; i < 2; ++i)
        {
            if (g_zero_point_done[i] == false)
            {
                if (g_zero_point_done[i] == 0)
                {
                    if (ms_count % 18 <= 9)
                    {
                        zero_point_pwm(pwm, i, -1);
                    }
                    else
                    {
                        zero_point_pwm(pwm, i, 1);
                    }
                    ms_count++;
                }
            }
            else
            {
                calc_encoder(curr_cnt, curr_rev, i);              /* エンコーダに関する計算 */
                calc_pwm(curr_cnt, angle_diff, curr_rev, pwm, i); /* pwm を計算 */
                if (g_param[PARAM_ERROR_STOP][i])
                    check_pol(angle_diff, pwm, i); /* エンコーダの極性の確認 */
            }
        }
        operate_motor(pwm); /* モータを操作 */

        /* 送信 */
        if (g_tx_flag)
        {
            TxData0[0] = UP(angle_diff[0]);
            TxData0[1] = LOW(angle_diff[0]);
            TxData0[2] = UP(angle_diff[1]);
            TxData0[3] = LOW(angle_diff[1]);
            TxData0[4] = (int8_t)pwm[0];
            TxData0[5] = (int8_t)pwm[1];
            TxData0[6] = (int8_t)ZERO_POINT1;
            TxData0[7] = (int8_t)ZERO_POINT2;
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
 * ピン変化割り込み
 *
-----------------------------------------------*/
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt()
{
    CNIF = 0;

    if (ZERO_POINT1 & !g_zero_point_done[0])
    {
        /* モータ停止 */
        OC1RS = OC_PERIOD;
        MOTOR_F1 = 0;
        MOTOR_B1 = 0;

        /* クリア */
        g_qei_int_cnt[0] = CLEAR;
        POS1CNT = CLEAR;

        /* フラグセット */
        g_zero_point_done[0] = true;
    }
    if (ZERO_POINT2 & !g_zero_point_done[1])
    {
        /* モータ停止 */
        OC2RS = OC_PERIOD;
        MOTOR_F2 = 0;
        MOTOR_B2 = 0;

        /* クリア */
        g_qei_int_cnt[1] = CLEAR;
        POS2CNT = CLEAR;

        /* フラグセット */
        g_zero_point_done[1] = true;
    }
}

/*-----------------------------------------------
 *
 * 原点合わせ時の PWM
 *
-----------------------------------------------*/
void zero_point_pwm(double pwm[], size_t ch, double sign)
{
    /* エラーチェック */
    if (ch != 0 && ch != 1)
        inform_err(2);

    pwm[ch] = g_param[PARAM_ZERO_POINT_PWM][ch] * sign;

    /* PARAM_ENABLE が無効だったらモータを駆動しない */
    if (g_param[PARAM_ENABLE][ch] == false)
        pwm[ch] = 0;
}

/*-----------------------------------------------
 *
 * エンコーダに関する計算
 *
-----------------------------------------------*/
void calc_encoder(int64_t curr_cnt[], double curr_rev[], size_t ch)
{
    /* エラーチェック */
    if (ch != 0 && ch != 1)
        inform_err(2);

    /* 現在の位置カウントを計算 */
    if (ch == 0)
        curr_cnt[0] = (int64_t)(65536 * g_qei_int_cnt[0] + POS1CNT);
    else
        curr_cnt[1] = (int64_t)(65536 * g_qei_int_cnt[1] + POS2CNT);

    /* エンコーダの極性に合わせて反転 */
    if (g_param[PARAM_ENCODER_POL][ch] == 1)
        curr_cnt[ch] *= -1;

    /* 回転数を計算 */
    calc_rev(curr_cnt, curr_rev, ch);
}

/*-----------------------------------------------
 *
 * 回転数を計算
 *
-----------------------------------------------*/
void calc_rev(int64_t curr_cnt[], double curr_rev[], size_t ch)
{
    static int64_t pre_cnt[2]; /* 前回の位置カウント */

    /* 回転数を求めるときに使う係数を計算 */
    const double coef[2] = {
        1 / (g_param[PARAM_ENCODER_RESOLUTION][0] * 4.0) / (CALC_PERIOD * 1E-3) * 60.0,
        1 / (g_param[PARAM_ENCODER_RESOLUTION][1] * 4.0) / (CALC_PERIOD * 1E-3) * 60.0,
    };

    /* 回転数を計算 */
    curr_rev[ch] = (curr_cnt[ch] - pre_cnt[ch]) * coef[ch];

    /* 前回の位置カウントを更新 */
    pre_cnt[ch] = curr_cnt[ch];
}

/*-----------------------------------------------
 *
 * PWM を計算
 *
-----------------------------------------------*/
void calc_pwm(int64_t curr_cnt[], int16_t angle_diff[], double curr_rev[], double pwm[], size_t ch)
{
    static int64_t old_cnt_diff[2]; /* 前の回転角偏差 */
    static double ie[2];
    double de;
    /* エラーチェック */
    if (ch != 0 && ch != 1)
        inform_err(2);

    /* 位置カウント偏差を計算 */
    int64_t cnt_diff = calc_cnt_diff(curr_cnt, angle_diff, ch);

    de = (cnt_diff - old_cnt_diff[ch]) / CALC_PERIOD;

    ie[ch] = ie[ch] + (cnt_diff + old_cnt_diff[ch]) * CALC_PERIOD / 2;

    /* 比例制御 */
    pwm[ch] = g_param[PARAM_KP][ch] * 1E-3 * cnt_diff +
              g_param[PARAM_KD][ch] * 1E-1 * -de +
              g_param[PARAM_KI][ch] * 1E-5 * ie[ch] +
              g_param[PARAM_MIN_PWM][ch] * GET_SIGNAL_INT(cnt_diff);
    pwm[ch] = (fabs(pwm[ch]) > g_param[PARAM_MAX_PWM][ch])
                  ? g_param[PARAM_MAX_PWM][ch] * GET_SIGNAL_FLOAT(pwm[ch])
                  : pwm[ch];

    /* -100 ～ 100 に収める */
    pwm[ch] = (fabs(pwm[ch]) > 100)
                  ? 100 * GET_SIGNAL_FLOAT(pwm[ch])
                  : pwm[ch];

    /* pwm が大きくても curr_rev が小さい場合，
        駆動が OFF であるとみなして pwm の上限を stop_pwm に制限する */
    if (fabs(curr_rev[ch]) < g_param[PARAM_STOP_REV][ch] && fabs(pwm[ch]) > g_param[PARAM_STOP_PWM][ch])
        pwm[ch] = g_param[PARAM_STOP_PWM][ch] * GET_SIGNAL_FLOAT(pwm[ch]);

    /* ある程度回転数が小さいときに偏差が許容誤差に収まったら停止 */
    if (abs(angle_diff[ch]) < g_param[PARAM_PERMISSIBLE_ERR][ch])
        pwm[ch] = 0;

    /* 加減速処理 */
    smoothen_pwm_change(pwm, ch);

    /* kp が 0 のときは停止 */
    if (g_param[PARAM_ENABLE][ch] == false)
        pwm[ch] = 0;
}

/*-----------------------------------------------
 *
 * 位置カウント偏差を計算
 *
-----------------------------------------------*/
int64_t calc_cnt_diff(int64_t curr_cnt[], int16_t angle_diff[], size_t ch)
{
    /* エラーチェック */
    if (ch != 0 && ch != 1)
        inform_err(2);

    /* 目標値を回転角から位置カウントに変換 */
    int64_t target_cnt = g_target_angle[ch] / 360.0 * g_param[PARAM_ENCODER_RESOLUTION][ch] * 4.0;

    /* 位置カウント偏差を計算 */
    int64_t cnt_diff = target_cnt - curr_cnt[ch];

    /* 位置カウント偏差を回転角偏差に変換 */
    angle_diff[ch] = cnt_diff / (g_param[PARAM_ENCODER_RESOLUTION][ch] * 4.0) * 360.0;

    return cnt_diff;
}

/*-----------------------------------------------
 *
 * 加減速処理
 *
-----------------------------------------------*/
void smoothen_pwm_change(double pwm[], size_t ch)
{
    const double max_change[2] = {
        g_param[PARAM_PWM_CHANGE][0] * CALC_PERIOD * 1E-3,
        g_param[PARAM_PWM_CHANGE][1] * CALC_PERIOD * 1E-3,
    };

    static double pre_pwm[2]; /* 前回の PWM */

    /* 変化量を計算 */
    double pwm_change = pwm[ch] - pre_pwm[ch];

    /* 変化量が大きかったら閾値に収める */
    if (fabs(pwm_change) > max_change[ch])
        pwm[ch] = pre_pwm[ch] + max_change[ch] * GET_SIGNAL_FLOAT(pwm_change);

    /* 更新 */
    pre_pwm[ch] = pwm[ch];
}

/*-----------------------------------------------
 *
 * エンコーダの極性を確認する
 *
-----------------------------------------------*/
void check_pol(int16_t angle_diff[], double pwm[], size_t ch)
{
    static size_t err_cnt[2];
    static int16_t pre_angle_diff[2];
    static double pre_pwm[2];

    /* エラーチェック */
    if (ch != 0 && ch != 1)
        inform_err(2);

    /* 偏差の変化量 */
    int16_t diff_change = abs(angle_diff[ch]) - abs(pre_angle_diff[ch]);

    /* 最大 PWM 出力時に回転角偏差が大きくなったら */
    bool c1 = (diff_change > 0);
    bool c2 = (g_param[PARAM_ENABLE][ch] == true);
    bool c3 = (fabs(pwm[ch]) == g_param[PARAM_MAX_PWM][ch]);

    if (c1 & c2 & c3)
        ++err_cnt[ch];
    else
        err_cnt[ch] = CLEAR;

    /* モータの回転方向が変わったら */
    if (pwm[ch] * pre_pwm[ch] < 0)
        err_cnt[ch] = CLEAR;

    /* 連続で条件を満たしたら停止 */
    if (err_cnt[ch] > 3)
        inform_err(ch);

    /* 更新 */
    pre_pwm[ch] = pwm[ch];
    pre_angle_diff[ch] = angle_diff[ch];
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
    else if (ch == 1)
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
    else
    {
        while (1)
        {
            debug_LED1 = ON;
            debug_LED2 = ON;
            __delay_ms(100);
            debug_LED1 = OFF;
            debug_LED2 = OFF;
            __delay_ms(100);
            debug_LED1 = ON;
            debug_LED2 = ON;
            __delay_ms(100);
            debug_LED1 = OFF;
            debug_LED2 = OFF;
            __delay_ms(400);
        }
    }
}
