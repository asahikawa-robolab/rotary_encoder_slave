/*
 * File:   communication.h
 * Author: nemoto
 *
 * Created on 2018/05/27, 9:33
 */

#ifndef COMMUNICATION_H
#define	COMMUNICATION_H

////フラグbit////
#define NORD number_of_rxdata0                  ////data数(RxDataxの配列数)////
#define NOB NORD * 2 + 3                        ////data受信数(Bufferxの配列数)////
#define enablebit_from_master IEC0bits.U1RXIE   ////受信許可ビット////
#define flagbit_from_master IFS0bits.U1RXIF     ////受信フラグビット////
#define REG_from_master U1RXREG                 ////受信レジスタ////
#define Buffer_from_master Buffer0              ////Buffer////
#define Data_from_master RxData0                ////RxData////
#define FERR_from_master U1STAbits.FERR         ////フレーミングエラーフラグ////
#define OERR_from_master U1STAbits.OERR         ////オーバーランエラーフラグ////
//#define CREN_from_master RC2STAbits.CREN        ////連続的な受信許可ビット////

#define T2_TRIS TRISDbits.TRISD1

////バッファ////
#define ADD_upper Buffer0[1].upper_data
#define ADD_lower Buffer0[2].upper_data

////スレーブアドレス////
/*
スレーブアドレスはそれぞれ変えてください
モータは下位ビットを、電磁弁とサーボは上位ビットを設定してください
 */
#define Motor_slave_address (PORTC & 0x0f)
#define Encoder_slave_Address 0xe0

////受信用ステータス////
#define reception_complete 0 //受信完了(受信待機中)
#define waiting_to_receive 1 //受信中
////受信用使い分け////
#define master_COM 0

////slave_check関数用ステータス
#define continuation 0
#define slave_match 1

#define not_transmit 0
#define match_and_transmit 1


////送信用ステータス////
#define send_start 0
#define send_till 1
#define send_stop 2


extern bool Receive_flag;

extern bool Reception_from_master(uint8_t); //slave→master
extern void Reception_from_master_main(void);
extern void transmit_for_master(bool);

extern void Flag_reset(void);

#endif	/* COMMUNICATION_H */