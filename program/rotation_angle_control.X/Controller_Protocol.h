#ifndef Controller_Protocol_H
#define	Controller_Protocol_H

#include "Controller_Protocol.h"
#include <stdbool.h>

typedef struct {
    int num;
    int count;
} Organize_Parameter;

typedef struct {
    int Buffer_count;
    bool Start_signal_has_come;
    bool S_flag;
} Store_Parameter;

typedef struct {
    int data_send_count;
    int last_signal;
    bool byte_data_complete;
} Send_Parameter;

typedef union {

    struct {
        unsigned lower_data : 4;
        unsigned upper_data : 4;
    };
    unsigned char all_data;
} Untreated;

typedef union {

    struct {
        unsigned d0 : 1;
        unsigned d1 : 1;
        unsigned d2 : 1;
        unsigned d3 : 1;
        unsigned d4 : 1;
        unsigned d5 : 1;
        unsigned d6 : 1;
        unsigned d7 : 1;
    };

    struct {
        unsigned lower_data : 4;
        unsigned upper_data : 4;
        unsigned LD_error : 1;
        unsigned UD_error : 1;
    };

    struct {
        unsigned char all_data;
        unsigned errors : 2;
    };
} Processed;

/*受信用記憶領域確保*/
#define number_of_rxdata0 5
//#define number_of_rxdata1 20
//#define number_of_rxdata2 8

/*送信用記憶領域確保*/
#define number_of_txdata0 6
//#define number_of_txdata1 8
//#define number_of_txdata2 8

/*通信バス数指定*/
#define rx_number 2
#define tx_number 2

#define error 1
#define not_error 0
#define complete 2

#ifdef number_of_rxdata0
extern Untreated Buffer0[number_of_rxdata0 * 2 + 3];
extern Processed RxData0[number_of_rxdata0];
#endif
#ifdef number_of_rxdata1
extern Untreated Buffer1[number_of_rxdata1 * 2 + 3];
extern Processed RxData1[number_of_rxdata1];
#endif
#ifdef number_of_rxdata2
extern Untreated Buffer2[number_of_rxdata2 * 2 + 3];
extern Processed RxData2[number_of_rxdata2];
#endif

#ifdef number_of_txdata0
extern unsigned char TxData0[number_of_txdata0];
#endif
#ifdef number_of_txdata1
extern unsigned char TxData1[number_of_txdata1] = {0, 0x01, 0x02, 0x34, 0x33, 0x16, 0x50, 0xAB};
#endif
#ifdef number_of_txdata2
extern unsigned char TxData2[number_of_txdata2];
#endif

extern void Initialize_Parameters(void);
extern void Organize_Datas(Processed *, Untreated *, int, int);
extern unsigned char Store_Datas(Untreated *, unsigned char, int, int, bool (*)(void));
extern void Send_StartSignal(void (*)(unsigned char), void (*)(bool), bool);
extern bool Send_Till_EndSignal(unsigned char *, void (*)(unsigned char), void (*)(bool), int, int);

#ifdef	__cplusplus
#endif
#ifdef	__cplusplus
#endif
#endif