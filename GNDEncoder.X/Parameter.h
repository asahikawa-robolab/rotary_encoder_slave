#ifndef PARAMETER_H
#define PARAMETER_H

/* モード */
#define MODE_NUM 2
#define MODE_A 0
#define MODE_B 1

/* エンコーダ分解能 */
#define NEW 384
#define OLD 300

/*-----------------------------------------------
 *
 * モード毎の設定
 *
-----------------------------------------------*/
/* A チーム */
#define A_ENCODER_RESOLUTION NEW
#define A_POL_INVERSE_X true
#define A_POL_INVERSE_Y false

/* B チーム */
#define B_ENCODER_RESOLUTION NEW
#define B_POL_INVERSE_X true
#define B_POL_INVERSE_Y false

const double EncoderResolution[MODE_NUM] = {
    A_ENCODER_RESOLUTION, B_ENCODER_RESOLUTION
};
const double PolInverse[MODE_NUM][2] = {
    {A_POL_INVERSE_X, A_POL_INVERSE_Y},
    {B_POL_INVERSE_X, B_POL_INVERSE_Y}
};

#endif