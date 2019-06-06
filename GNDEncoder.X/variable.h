/* 
 * File:   variable.h
 * Author: nemoto
 *
 * Created on 2018/04/19, 17:28
 */

#ifndef VARIABLE_H
#define	VARIABLE_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* VARIABLE_H */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <xc.h>

#include "Controller_Protocol.h"


#define _XTAL_FREQ  16000000


//////////////////////////////Motor Data受信//////////////////////////////
#define SLAVE_ADDRESS RxData0[0].all_data

#define rx_sensor0 RxData0[1].d0
#define rx_sensor1 RxData0[1].d1
#define rx_sensor2 RxData0[1].d2
#define rx_sensor3 RxData0[1].d3
#define rx_sensor4 RxData0[1].d4
#define rx_sensor5 RxData0[1].d5
#define rx_sensor6 RxData0[1].d6
#define rx_sensor7 RxData0[1].d7

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////I/Oピンの名前//////////////////////////////

////////////////////////////////////////////////////////////////////////////////

//////////////////////////////フラグレジスタの名前//////////////////////////////
#define T_enable IEC0bits.U1TXIE
#define T_flag IFS0bits.U1TXIF

#define R_enable IEC0bits.U1RXIE
#define R_flag IFS0bits.U1RXIE

#define NOADC 4

////////////////////////////////////////////////////////////////////////////////

//基本フラグ
#define on 1
#define off 0

#define enable 1
#define disable 0

#define set 1
#define clear 0
