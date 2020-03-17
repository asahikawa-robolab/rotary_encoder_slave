/* version 2020.03.17 */
#ifndef GENERAL_H
#define GENERAL_H

#include <stdbool.h>
#include <stdint.h>
#include "Controller_Protocol.h"

/*-----------------------------------------------
 *
 * define
 *
-----------------------------------------------*/
#define ON 1
#define OFF 0
#define enable 1
#define disable 0
#define CLEAR 0

/* 通信用 */
#define UP(data) ((((int16_t)data) >> 8) & 0xFF)
#define LOW(data) (((int16_t)data) & 0xFF)
#define ASBL(up, low) ((int16_t)((up) << 8) | (low)) /* Assemble */

/* 符号を返す */
#define GET_SIGNAL_INT(x) ((x) == 0 ? 0 : (x) / abs(x))    /* 整数用 */
#define GET_SIGNAL_FLOAT(x) ((x) == 0 ? 0 : (x) / fabs(x)) /* 実数用 */

/* パラメータ設定 */
#define MAX_NUM_OF_PARAM 254 /* パラメータの最大個数 */
#define PARAM_UNIT_NUM 2     /* 同時に読み込むパラメータの数 */

/*-----------------------------------------------
 *
 * エラーチェック
 *
-----------------------------------------------*/
#if (PARAM_UNIT_NUM != 2)
#error "invalid PARAM_UNIT_NUM"
#endif
#if (number_of_rxdata0 < PARAM_UNIT_NUM * 2 + 1)
#error "invalid number_of_rxdata0"
#endif
#if (number_of_txdata0 < 1)
#error "invalid number_of_txdata0"
#endif

/*-----------------------------------------------
 *
 * 宣言
 *
-----------------------------------------------*/
extern bool g_tx_flag;
void EUSART_Write(unsigned char data);
void EUSART_TxInterrupt_Control(bool enable_or_disable);
bool EUSART_ERROR_from_master(void);
void load_param(int16_t param[][PARAM_UNIT_NUM]);
void operate_motor(double pwm[]);
void apply_pwm(double pwm[]);
void apply_io(double pwm[]);

#endif