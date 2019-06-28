/*
 * File:   communication.c
 * Author: nemoto
 *
 * Created on 2018/05/27, 9:32
 */
#include "variable.h"
#include "communication.h"
#include "Controller_Protocol.h"
#include <xc.h>


#define debug_LED1 LATBbits.LATB2
#define debug_LED2 LATBbits.LATB3
/*
 *
 */

bool Receive_flag = waiting_to_receive;

bool slave_different_flag = continuation;
bool slave_transmit_flag = not_transmit;

bool EUSART_ERROR_from_master(void);
bool Reception_from_master(uint8_t);
bool slave_address_check(uint8_t, uint8_t, uint8_t);
void Reception_from_master_main(void);

void transmit_for_master(bool);
void EUSART_Write_for_master(unsigned char);
void EUSART_TxInterrupt_Control_for_master(bool);

void Flag_reset(void);

/*debug*/
int debug_buf;
 unsigned char Rx_buf;
//受信の割り込みを許可

bool EUSART_ERROR_from_master(void) {
    if ((FERR_from_master == 1) || (OERR_from_master == 1)) {
        //フレーミングエラー処理
        uint8_t waste_data;
        waste_data = REG_from_master;
        //オーバーランエラー処理
        U1MODE = 0x0808;
        U1STA = 0x2000;
        U1MODEbits.UARTEN = 1;
        U1STAbits.UTXEN = 1;
        return error;
    }

    return not_error;
}

bool Reception_from_master(uint8_t slave_address) {
    static uint8_t Receive_data_count = 0;
    uint8_t error_complete = 0;
    /*debug*/


    //    error_complete = Store_Datas(Buffer_from_master, REG_from_master, NOB, master_COM, EUSART_ERROR_from_master());
    //
    //    if (error_complete == error) {
    //        Receive_data_count = clear;
    //    } else {
    //        Receive_data_count++;
    //
    //        slave_different_flag = slave_address_check(Receive_data_count, slave_address, slave_different_flag);
    //    }
    //
    //
    //
    //    if (error_complete == complete) {
    //        Receive_flag = reception_complete;
    //        Receive_data_count = clear;
    //        //Timer_Reload();
    //        //Timer_Start();
    //    }

    Rx_buf = REG_from_master;  //この一文消すな！！！

    /*debug*/
//     U2TXREG = Rx_buf;
//    while (U2STAbits.UTXBF);
//    IFS1bits.U2TXIF = enable;
    /*debug*/

    error_complete = Store_Datas(Buffer_from_master, Rx_buf, NOB, master_COM, EUSART_ERROR_from_master);

    if (error_complete == error) {
        Receive_data_count = clear;

    } else if ((error_complete == complete) && (Receive_data_count != (NOB - 1))) {
        Receive_data_count = clear;
    } else {
        slave_different_flag = slave_address_check(Receive_data_count, slave_address, slave_different_flag);
        Receive_data_count++;
    }

    if ((error_complete == complete) && (Receive_data_count == NOB)) {

        Receive_flag = reception_complete;
        Receive_data_count = clear;
    }

    return Receive_flag;

}

bool slave_address_check(uint8_t data_count, uint8_t slave_address, uint8_t slave_flag) {

    //    static bool continuous_check = 0;

    if (slave_flag == continuation) {

        if (data_count == 3) {
            uint8_t sample_data;

            sample_data = (ADD_upper << 4) | ADD_lower;

            if (sample_data == slave_address) {
                //                slave_transmit_flag = match_and_transmit;
                //                continuous_check = 0;

                if ((sample_data & 0x08) != 0) {
                    slave_transmit_flag = match_and_transmit;
                    //                    continuous_check = 1;
                }

                return slave_match;
            }

        }

        return continuation;
    } else {
        return slave_flag;
    }
}

void Reception_from_master_main(void) {
    if (Receive_flag == reception_complete) {
        if (slave_different_flag == slave_match) {
            Organize_Datas(Data_from_master, Buffer_from_master, number_of_rxdata0, master_COM);
        }
        slave_different_flag = continuation;
        Receive_flag = waiting_to_receive;
    }
}

void transmit_for_master(bool start_endtill) {
    //    static uint8_t TxData_number = 0;
    bool send_till_flag = 0;
    static bool send_complete_flag = enable;

    if (slave_transmit_flag == match_and_transmit) {
        if ((start_endtill == send_start) && (send_complete_flag == enable)) {
            send_complete_flag = disable;
            Send_StartSignal(EUSART_Write_for_master, EUSART_TxInterrupt_Control_for_master, T_enable);
        } else if (start_endtill == send_till) {
            send_till_flag = Send_Till_EndSignal(TxData0, EUSART_Write_for_master, EUSART_TxInterrupt_Control_for_master, number_of_txdata0, 0);
            if (send_till_flag == true) {
                slave_transmit_flag = not_transmit;
                send_complete_flag = enable;
            }
        }
    }
}

void EUSART_Write_for_master(unsigned char txData) {
    //配列 txDataの先頭アドレスを渡す
    U1TXREG = txData;
}

//送信の割り込み許可

void EUSART_TxInterrupt_Control_for_master(bool enable_or_disable) {
    //Transmit_enable = enable_or_disable;
    T_enable = enable_or_disable;
}

void Flag_reset(void) {
    slave_different_flag = continuation;
}
