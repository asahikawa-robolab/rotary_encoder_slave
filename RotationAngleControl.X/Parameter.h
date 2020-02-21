#ifndef PARAMETER_H
#define PARAMETER_H

#include "Config.h"

/* アドレス */
#define MY_ADDRESS 0x90

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
#define MODE_NUM 6
/* テスト */
/* A 自動 T シャツ */
/* A 自動昇降 下，上 */
/* A 手動昇降 */
/* B 自動昇降 */
/* B 手動昇降 */

/* エンコーダの極性 */
const bool EncoderPol[MODE_NUM][2] = {
    {REVERSE, 0},
    {FORWARD, 0},
    {REVERSE, REVERSE},
    {REVERSE, 0},
    {REVERSE, 0},
    {REVERSE, 0},
};
/* エンコーダの種類 */
const double EncoderKind[MODE_NUM][2] = {
    {OLD, 0},
    {OLD, 0},
    {OLD, OLD},
    {OLD, 0},
    {NEW, 0},
    {NEW, 0},
};
/* RPM → PWM の比例ゲイン */
const double KpRev[MODE_NUM][2] = {
    {0.5, 0},
    {0.5, 0},
    {0.07, 0.07},
    {0.5, 0},
    {0.15, 0},
    {0.15, 0},
};
/* angle → RPM の比例ゲイン */
const double KpAngle[MODE_NUM][2] = {
    {0.1, 0},
    {0.1, 0},
    {0.2, 0.2},
    {0.1, 0},
    {0.1, 0},
    {0.1, 0},
};
/* ゼロ点合わせ時の回転数 */
const double OriginRPM[MODE_NUM][2] = {
    {-30, 0},
    {-5, 0},
    {40, 40},
    {10, 0},
    {10, 0},
    {10, 0},
};
/* 位置合わせの微調整を開始する偏差の絶対値 */
const double SmallDif[MODE_NUM][2] = {
    {10, 0},
    {10, 0},
    {15, 15},
    {10, 0},
    {10, 0},
    {10, 0},
};
/* 偏差が SMALL_DIF 以下になったときの回転数 */
const double SmallDifRPM[MODE_NUM][2] = {
    {1, 0},
    {1, 0},
    {15, 15},
    {1, 0},
    {1, 0},
    {1, 0},
};
/* 収束したと判定する許容誤差 */
const double PermissibleError[MODE_NUM][2] = {
    {2, 0},
    {10, 0},
    {40, 40},
    {2, 0},
    {2, 0},
    {5, 0},
};
/* 収束判定の条件．偏差を連続で PermissionError 以下にする回数 */
const double ConvergeCnt[MODE_NUM][2] = {
    {5, 0},
    {5, 0},
    {2, 2},
    {5, 0},
    {5, 0},
    {5, 0},
};
/* モータが停止したと判定する回転数 */
const double StopRPM[MODE_NUM][2] = {
    {3, 0},
    {3, 0},
    {10, 5},
    {3, 0},
    {3, 0},
    {3, 0},
};
/* モータが停止したと判断されたときに出力する PWM の最大値 */
const double StopPWM[MODE_NUM][2] = {
    {10, 0},
    {40, 0},
    {30, 30},
    {15, 0},
    {25, 0},
    {25, 0},
};
/* 位置合わせ時の回転数の最大値 */
const double MaxRPM[MODE_NUM][2] = {
    {50, 0},
    {7, 0},
    {80, 80},
    {50, 0},
    {20, 0},
    {15, 0},
};
/* 全体を通して出力するデューティー比の最大値 */
const double MaxPWM[MODE_NUM][2] = {
    {50, 0},
    {50, 0},
    {50, 50},
    {50, 0},
    {50, 0},
    {50, 0},
};

#endif