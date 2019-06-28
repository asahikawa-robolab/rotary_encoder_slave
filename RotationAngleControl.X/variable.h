#ifndef VARIABLE_H
#define VARIABLE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <xc.h>
#include "Controller_Protocol.h"

#define _XTAL_FREQ 16000000

/*-----------------------------------------------
 *
 * 通信フラグ
 *
-----------------------------------------------*/
#define T_enable IEC0bits.U1TXIE
#define T_flag IFS0bits.U1TXIF
#define R_enable IEC0bits.U1RXIE
#define R_flag IFS0bits.U1RXIE

/*-----------------------------------------------
 *
 * 基本フラグ
 *
-----------------------------------------------*/
#define on 1
#define off 0
#define ON 1
#define OFF 0
#define enable 1
#define disable 0
#define set 1
#define clear 0
#define true 1
#define false 0

#endif