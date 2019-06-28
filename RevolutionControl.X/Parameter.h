#ifndef PARAMETER_H
#define PARAMETER_H

#include "Config.h"

/* アドレス */
#define MY_ADDRESS 0x00

/* エンコーダの極性 */
#define FORWARD 0
#define REVERSE 1

/* エンコーダの種類 */
#define NEW 384
#define OLD 300

/*-----------------------------------------------
 *
 * モード毎の設定
 *
-----------------------------------------------*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 ! 使用しないチャンネルの値は 0 にすること !
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
#define MODE_NUM 4
/* テスト */
/* A 自動手動 */
/* B 自動 */
/* B 手動 */

/* エンコーダの極性 */
const bool EncoderPol[MODE_NUM][2] = {
    {REVERSE, 0},
    {FORWARD, FORWARD},
    {FORWARD, FORWARD},
    {FORWARD, FORWARD},
};
/* エンコーダの種類 */
const double EncoderKind[MODE_NUM][2] = {
    {OLD, 0},
    {NEW, NEW},
    {NEW, NEW},
    {NEW, NEW},
};
/* モータが停止したと判断する回転数 */
const double StopRPM[MODE_NUM][2] = {
    {3, 0},
    {3, 3},
    {3, 3},
    {3, 3},
};
/* モータが停止したと判断されたときに出力される PWM の最大値 */
const double StopPWM[MODE_NUM][2] = {
    {10, 0},
    {10, 10},
    {10, 10},
    {10, 10},
};
/* モータの最高速度 */
const double MaxRPM[MODE_NUM][2] = {
    {50, 0},
    {300, 300},
    {200, 200},
    {200, 200},
};
/* 比例ゲイン */
const double Kp[MODE_NUM][2] = {
    {0.5, 0},
    {0.1, 0.1},
    {0.1, 0.1},
    {0.08, 0.08},
};

#endif