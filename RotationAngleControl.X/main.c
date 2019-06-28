/*
 * File:   main.c
 * Author: pguz1
 *
 * Created on 2019/01/02, 22:38
 */
/*回転角回転数制御*/
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <p33FJ128MC802.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "Controller_Protocol.h"
#include "communication.h"
#include "variable.h"
#include "Config.h"
#include "Parameter.h"

typedef enum
{
    F1,
    F2,
    B1,
    B2,
    ALL,
} MOTOR_IO_NAME;

/*-----------------------------------------------
 * プロトタイプ宣言
-----------------------------------------------*/
void Initialize();
void SetOrigin(MOTOR Motor[], int pre_QEI_IntCnt[], POSCNTS PosCnts[]);
void MeasureRPM(MOTOR Motor[], int pre_QEI_IntCnt[], POSCNTS PosCnts[]);
void CalcPWM(MOTOR Motor[], POSCNTS PosCnts[]);
void CalcPWM_Normal(MOTOR Motor[], POSCNTS PosCnts[], int i);
void CalcPWM_Origin(MOTOR Motor[], int i);
double CalcMV(MOTOR Motor[], int i);
void SetMotorIO(MOTOR_IO_NAME name, bool value);
void OperateMotor(MOTOR Motor[]);
void StopMotor(void);
void CheckTargetChange(bool flag[], POSCNTS PosCnts[]);
void CheckConverge(MOTOR Motor[]);
void CheckPol(MOTOR Motor[]);
void Error(void);
double Sign(double value);
void __attribute__((interrupt, no_auto_psv)) _QEI1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _QEI2Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(); /* ピン変化割り込み（ゼロ点合わせ） */
void EUSART_Write(unsigned char data);
void EUSART_TxInterrupt_Control(bool enable_or_disable);

/*-----------------------------------------------
 * グローバル変数
-----------------------------------------------*/
/* 通信フラグ */
bool TxFrag = 1;
bool rx_has_started = 0; /* 受信開始したフラグ */

/* マスターからの受信データ */
int16_t TargetAngle[2] = {0, 0}; /* 回転角の目標値 */
uint8_t Mode;                    /* 動かす機構の種類 */
bool Stop;                       /* モータを停止させる */

/* フラグ */
#define ORIGIN_NO 0                        /* ピン変化割り込み発生前 */
#define ORIGIN_WAIT 1                      /* ピン変化割り込み発生後で，初期化処理実行前 */
#define ORIGIN_OK 2                        /* ゼロ点合わせ完了 */
int Origin_OK[2] = {ORIGIN_NO, ORIGIN_NO}; /* ゼロ点判定 */
#define CONVERGE_ZERO_POINT_ADJUSTING 0
#define CONVERGE_FALSE 1
#define CONVERGE_TRUE 2
int Converge_OK[2] = {CONVERGE_ZERO_POINT_ADJUSTING, CONVERGE_ZERO_POINT_ADJUSTING}; /* 収束判定 */

/* その他 */
int curr_QEI_IntCnt[2] = {0, 0}; /* QEI割り込み回数 */

double debug[2];

int main(int argc, char **argv)
{
    Initialize();

    while (1)
    {
        /*-----------------------------------------------
         * マスターから受信
        -----------------------------------------------*/
        if (Receive_flag == reception_complete)
        {
            Reception_from_master_main();

            if (rx_has_started == 0)
                rx_has_started = 1;

            TargetAngle[0] = ASBL(RxData0[1].all_data, RxData0[2].all_data);
            TargetAngle[1] = ASBL(RxData0[3].all_data, RxData0[4].all_data);
            Mode = RxData0[5].all_data & 0b01111111;
            Stop = RxData0[5].all_data >> 7;
            debug_LED1 ^= 1;
        }
        /*-----------------------------------------------
         * マスターに送信（デバッグ用）
        -----------------------------------------------*/
        if (TxFrag)
        {
            TxData0[0] = Converge_OK[0];
            TxData0[1] = Converge_OK[1];
            TxData0[2] = UP(debug[0]);
            TxData0[3] = LOW(debug[0]);
            TxData0[4] = UP(debug[1]);
            TxData0[5] = LOW(debug[1]);
            Send_StartSignal(EUSART_Write,
                             EUSART_TxInterrupt_Control,
                             IEC1bits.U2TXIE);

#if MY_ADDRESS == 0x00
            /* 送信周期調整用 */
            __delay_ms(10 * number_of_txdata0);
#endif
        }
    }
    return (EXIT_SUCCESS);
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;
    static int TmrIntCnt; /* タイマー割り込みの発生した回数をカウントする */
    static int pre_QEI_IntCnt[2];
    static MOTOR Motor[2];

    static POSCNTS PosCnts[2] = {
        {QEI_INIT_VALUE, QEI_INIT_VALUE, QEI_INIT_VALUE, QEI_INIT_VALUE},
        {QEI_INIT_VALUE, QEI_INIT_VALUE, QEI_INIT_VALUE, QEI_INIT_VALUE},
    };

    TmrIntCnt++;
    PR1 = PID_TIMER_1MS;

    /* PID_PERIOD [ms] 毎に真になる */
    if (TmrIntCnt >= PID_PERIOD)
    {
        if (rx_has_started == true)
        {
            /* 原点合わせ完了時の処理 */
            SetOrigin(Motor, pre_QEI_IntCnt, PosCnts);

            /* PosCnt を取得 */
            PosCnts[0].cur = POS1CNT;
            PosCnts[1].cur = POS2CNT;

            MeasureRPM(Motor, pre_QEI_IntCnt, PosCnts); /* 回転数を測定 */
            CalcPWM(Motor, PosCnts);                    /* モータへの出力を決定する */
            OperateMotor(Motor);                        /* モータへ出力する */
            CheckConverge(Motor);                       /* 収束判定 */
            CheckPol(Motor);                            /* エンコーダの極性が正しいか確認する */

            /* 更新 */
            PosCnts[0].pre = PosCnts[0].cur;
            PosCnts[1].pre = PosCnts[1].cur;
            pre_QEI_IntCnt[0] = curr_QEI_IntCnt[0];
            pre_QEI_IntCnt[1] = curr_QEI_IntCnt[1];

            /* debug */
            debug[0] = Motor[0].dif;
            debug[1] = Motor[0].pwm;
        }
        TmrIntCnt = clear;
    }
}

/*-----------------------------------------------
 *
 * ピン変化割り込み（ゼロ点合わせ）
 *
-----------------------------------------------*/
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt()
{
    IFS1bits.CNIF = 0;

    if (Limit_1 == true && Origin_OK[0] == ORIGIN_NO)
    {
        Origin_OK[0] = ORIGIN_WAIT; /* フラグセット */
        OC1RS = OC_PERIOD;          /* モータ停止 */
        SetMotorIO(F1, OFF);
        SetMotorIO(B1, OFF);
    }
    else if (Limit_2 == true && Origin_OK[1] == ORIGIN_NO)
    {
        Origin_OK[1] = ORIGIN_WAIT;
        OC2RS = OC_PERIOD;
        SetMotorIO(F2, OFF);
        SetMotorIO(B2, OFF);
    }
}

/*-----------------------------------------------
 *
 * 原点を設定する
 *
-----------------------------------------------*/
void SetOrigin(MOTOR Motor[], int pre_QEI_IntCnt[], POSCNTS PosCnts[])
{
    int i;
    for (i = 0; i < 2; i++)
    {
        if (Origin_OK[i] == ORIGIN_WAIT)
        {
            /* リセット */
            Motor[i].t_rpm = 0;
            Motor[i].pwm = 0;
            pre_QEI_IntCnt[i] = 0;
            curr_QEI_IntCnt[i] = 0;
            PosCnts[i].org = QEI_INIT_VALUE;
            PosCnts[i].pre = QEI_INIT_VALUE;
            PosCnts[i].cur = QEI_INIT_VALUE;
            if (i == 0)
                POS1CNT = QEI_INIT_VALUE;
            else if (i == 1)
                POS2CNT = QEI_INIT_VALUE;

            /* フラグセット */
            Origin_OK[i] = ORIGIN_OK;
        }
    }
}

/*-----------------------------------------------
 *
 * 回転数を測定する
 *
-----------------------------------------------*/
void MeasureRPM(MOTOR Motor[], int pre_QEI_IntCnt[], POSCNTS PosCnts[])
{
    /* 回転数を求めるときに掛け合わせる係数（coefficient） */
    const double COEF[2] = {
        1 / (EncoderKind[Mode][0] * 4.0) / (PID_PERIOD * 1E-3) * 60.0,
        1 / (EncoderKind[Mode][1] * 4.0) / (PID_PERIOD * 1E-3) * 60.0,
    };

    Motor[0].c_rpm = (int)((int)((65536 * curr_QEI_IntCnt[0] + PosCnts[0].cur) - (65536 * pre_QEI_IntCnt[0] + PosCnts[0].pre)) * COEF[0]);
    Motor[1].c_rpm = (int)((int)((65536 * curr_QEI_IntCnt[1] + PosCnts[1].cur) - (65536 * pre_QEI_IntCnt[1] + PosCnts[1].pre)) * COEF[1]);
}

/*-----------------------------------------------
 *
 * MD に出力するデューティー比を計算する
 *
-----------------------------------------------*/
void CalcPWM(MOTOR Motor[], POSCNTS PosCnts[])
{
    int i;
    for (i = 0; i < 2; i++)
    {
        /*-----------------------------------------------
         * POSCNT の偏差を計算
        -----------------------------------------------*/
        /* [deg]で受信した目標値を[count]に変換 */
        PosCnts[i].target = (TargetAngle[i] * (EncoderKind[Mode][i] * 4.0) / 360.0);
        /* PosCnt の偏差を計算 */
        Motor[i].dif = (PosCnts[i].target - (int16_t)(65536 * curr_QEI_IntCnt[i] + (int16_t)(PosCnts[i].cur - PosCnts[i].org)));

        /*-----------------------------------------------
         * PWM を計算
        -----------------------------------------------*/
        if (Origin_OK[i] == ORIGIN_OK)
            CalcPWM_Normal(Motor, PosCnts, i);
        else if (Origin_OK[i] == ORIGIN_NO)
            CalcPWM_Origin(Motor, i);

        /* PWM[] を -100~100 の範囲に収める */
        if (fabs(Motor[i].pwm) > 100)
            Motor[i].pwm = Sign(Motor[i].pwm) * 100;

        /* duty が大きくても RPM が小さい場合、
         * 駆動が OFF であるとみなしてPWMの大きさの上限を MinPWM に制限する */
        if (abs(Motor[i].c_rpm) < StopRPM[Mode][i] && fabs(Motor[i].pwm) > StopPWM[Mode][i])
            Motor[i].pwm = Sign(Motor[i].pwm) * StopPWM[Mode][i];
    }
}

void CalcPWM_Normal(MOTOR Motor[], POSCNTS PosCnts[], int i)
{
    /*-----------------------------------------------
     * 回転数の目標値を設定する
    -----------------------------------------------*/
    /* 偏差が小さかったら一定の回転数で */
    if (abs(Motor[i].dif) <= SmallDif[Mode][i])
        Motor[i].t_rpm = Sign(Motor[i].pwm) * SmallDifRPM[Mode][i];
    /* 偏差が大きかったら比例制御 */
    else
        Motor[i].t_rpm = (int)(Motor[i].dif * KpAngle[Mode][i]);

    /* TargetRPM の上限を MaxRPM */
    if (abs(Motor[i].t_rpm) > MaxRPM[Mode][i])
        Motor[i].t_rpm = MaxRPM[Mode][i] * Sign(Motor[i].t_rpm);

    /*-----------------------------------------------
     * デューティー比を計算
    -----------------------------------------------*/
    static bool change[2] = {false, false}; /* 回転角の目標値が変化したかどうか */
    CheckTargetChange(change, PosCnts);

    if (abs(Motor[i].dif) < PermissibleError[Mode][i])
    {
        Motor[i].pwm = 0;
    }
    /* 目標値が更新されてから初めて偏差が SmallDif 以下になったら一度停止 */
    else if (abs(Motor[i].dif) < SmallDif[Mode][i] && change[i] == true)
    {
        Motor[i].pwm = 0;
        change[i] = false;
    }
    else
    {
        Motor[i].pwm += CalcMV(Motor, i);
    }
}

void CalcPWM_Origin(MOTOR Motor[], int i)
{
    Motor[i].t_rpm = OriginRPM[Mode][i]; /* 回転数の目標値を設定 */
    Motor[i].pwm += CalcMV(Motor, i);    /* デューティー比を計算 */
}

/*-----------------------------------------------
 *
 * 操作量 MV を計算する
 *
-----------------------------------------------*/
double CalcMV(MOTOR Motor[], int i)
{
    return (Motor[i].t_rpm - Motor[i].c_rpm) * KpRev[Mode][i]; /* 操作量（Manipulated Variable）を計算 */
}

/*-----------------------------------------------
 *
 * MD の IOの操作
 *
-----------------------------------------------*/
void SetMotorIO(MOTOR_IO_NAME name, bool value)
{
    switch (name)
    {
    case F1:
        if (EncoderPol[Mode][0] == FORWARD)
            LATAbits.LATA2 = value;
        else
            LATBbits.LATB12 = value;
        break;
    case B1:
        if (EncoderPol[Mode][0] == FORWARD)
            LATBbits.LATB12 = value;
        else
            LATAbits.LATA2 = value;
        break;
    case F2:
        if (EncoderPol[Mode][1] == FORWARD)
            LATAbits.LATA3 = value;
        else
            LATBbits.LATB13 = value;
        break;
    case B2:
        if (EncoderPol[Mode][1] == FORWARD)
            LATBbits.LATB13 = value;
        else
            LATAbits.LATA3 = value;
        break;
    case ALL:
        LATAbits.LATA2 = value;
        LATAbits.LATA3 = value;
        LATBbits.LATB12 = value;
        LATBbits.LATB13 = value;
        break;
    default:
        Error();
    }
}

/*-----------------------------------------------
 *
 * モータを操作する
 *
-----------------------------------------------*/
void OperateMotor(MOTOR Motor[])
{
    /* ゲインが 0 のときは PWM を 0 に */
    if (KpAngle[0] == 0 || KpRev[0] == 0)
        Motor[0].pwm = 0;
    if (KpAngle[1] == 0 || KpRev[1] == 0)
        Motor[1].pwm = 0;

    /* Stop が true だったらモータを停止する */
    if (Stop)
    {
        StopMotor();
    }
    else
    {
        /* 反転して値を反映 */
        if (Origin_OK[0] != ORIGIN_WAIT)
            OC1RS = (uint16_t)(OC_PERIOD * (100 - fabs(Motor[0].pwm)) * 1E-2);
        if (Origin_OK[1] != ORIGIN_WAIT)
            OC2RS = (uint16_t)(OC_PERIOD * (100 - fabs(Motor[1].pwm)) * 1E-2);

        /* motor1 */
        if (Motor[0].pwm == 0)
        {
            SetMotorIO(F1, OFF);
            SetMotorIO(B1, OFF);
        }
        else if (Motor[0].pwm > 0)
        {
            SetMotorIO(F1, ON);
            SetMotorIO(B1, OFF);
        }
        else if (Motor[0].pwm < 0)
        {
            SetMotorIO(F1, OFF);
            SetMotorIO(B1, ON);
        }

        /* motor2 */
        if (Motor[1].pwm == 0)
        {
            SetMotorIO(F2, OFF);
            SetMotorIO(B2, OFF);
        }
        else if (Motor[1].pwm > 0)
        {
            SetMotorIO(F2, ON);
            SetMotorIO(B2, OFF);
        }
        else if (Motor[1].pwm < 0)
        {
            SetMotorIO(F2, OFF);
            SetMotorIO(B2, ON);
        }
    }
}

/*-----------------------------------------------
 *
 * モータを停止する
 *
-----------------------------------------------*/
void StopMotor(void)
{
    OC1RS = OC_PERIOD;
    OC2RS = OC_PERIOD;
    SetMotorIO(ALL, OFF);
}

/*-----------------------------------------------
 *
 * 目標値が変化したかを判定
 *
-----------------------------------------------*/
void CheckTargetChange(bool flag[], POSCNTS PosCnts[])
{
    static int32_t pre_target[2] = {0, 0};
    int i;

    for (i = 0; i < 2; i++)
    {
        if (pre_target[i] != PosCnts[i].target)
            flag[i] = true;
        pre_target[i] = PosCnts[i].target;
    }
}

/*-----------------------------------------------
 *
 * 収束判定
 *
-----------------------------------------------*/
void CheckConverge(MOTOR Motor[])
{
    static int cnt[2] = {0, 0};
    int i;

    for (i = 0; i < 2; i++)
    {
        /* 誤差が閾値に連続で収まった回数をカウント */
        if (abs(Motor[i].dif) < PermissibleError[Mode][i])
        {
            if (cnt[i] < ConvergeCnt[Mode][i])
                cnt[i]++;
        }
        else
        {
            cnt[i] = 0;
        }

        /* 連続回数が閾値を超えたら収束と判定する */
        if (Origin_OK[i] != ORIGIN_OK)
            Converge_OK[i] = CONVERGE_ZERO_POINT_ADJUSTING; /* ゼロ点合わせ未完了時 */
        else if (cnt[i] >= ConvergeCnt[Mode][i])
            Converge_OK[i] = CONVERGE_TRUE;
        else
            Converge_OK[i] = CONVERGE_FALSE;
    }
}

/*-----------------------------------------------
 *
 * 極性を確認．逆の場合は駆動を停止させる．
 *
-----------------------------------------------*/
void CheckPol(MOTOR Motor[])
{
    static int err_cnt[2] = {0, 0};
    int i;
    for (i = 0; i < 2; i++)
    {
        if (fabs(Motor[i].pwm) > MaxPWM[Mode][i])
            err_cnt[i]++;
        else
            err_cnt[i] = 0;

        if (err_cnt[i] > 3 && KpRev[i] != 0 && KpAngle[i] != 0)
            Error();
    }
}

void Error(void)
{
    /* モータを停止 */
    SetMotorIO(ALL, OFF);

    /* LED を光らせてエラーを知らせる */
    while (1)
    {
        ErrorLED = ON;
        __delay_ms(1000);
        ErrorLED = OFF;
        __delay_ms(1000);
        ErrorLED = ON;
        __delay_ms(1000);
        ErrorLED = OFF;
        __delay_ms(4000);
    }
}

/*-----------------------------------------------
 *
 * 符号を返す
 *
-----------------------------------------------*/
double Sign(double value)
{
    if (value == 0)
        return 0;
    else if (value > 0)
        return 1;
    else
        return -1;
}

////////////////////////////////////////

void EUSART_Write(unsigned char data)
{

    U2TXREG = data;
}

void EUSART_TxInterrupt_Control(bool enable_or_disable)
{

    IEC1bits.U2TXIE = enable_or_disable;
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt()
{
    Reception_from_master(MY_ADDRESS);
    IFS0bits.U1RXIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt()
{
    IFS1bits.U2TXIF = disable;

    TxFrag = Send_Till_EndSignal(TxData0,
                                 EUSART_Write,
                                 EUSART_TxInterrupt_Control,
                                 number_of_txdata0,
                                 bus1);
}

void __attribute__((interrupt, no_auto_psv)) _QEI1Interrupt()
{
    if (QEI1CONbits.UPDN)
        curr_QEI_IntCnt[0]++;
    else
        curr_QEI_IntCnt[0]--;
    IFS3bits.QEI1IF = clear;
}

void __attribute__((interrupt, no_auto_psv)) _QEI2Interrupt()
{
    if (QEI2CONbits.UPDN)
        curr_QEI_IntCnt[1]++;
    else
        curr_QEI_IntCnt[1]--;
    IFS4bits.QEI2IF = clear;
}

void Initialize()
{
    Initialize_Parameters();

    /*Oscillator*/
    OSCTUN = 18;            //7.84MHz
    PLLFBD = 40 - 2;        //M=40
    CLKDIVbits.PLLPRE = 0;  //PLL prescaler : /2  = N1
    CLKDIVbits.PLLPOST = 0; //PLL postscaler : /2 = N2
    //#pragma config FNOSC = FRC より Fin=7.37MHz(FRC)
    /* ↑ より Fosc=Fin*(M/(N1*N2)) =78.4MHz
     Fcy≒40MHz*/

    /*Initiate Clock Switch to Fast RC oscillator with PLL(NOSC=0b001)*/
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(0x01);

    /*Wait for Clock switch to occur
     * (current oscillator が0b001＝FRCでない間は待機)*/
    while (OSCCONbits.COSC != 0b001)
        ;

    /*Wait for PLL to lock*/
    while (OSCCONbits.LOCK != 1)
        ;

        /*Interrupt*/
#if 1                       //割り込みする:1 しない:0
    SRbits.IPL = 0;         // currPosCntU interrupt priority level:0
    CORCONbits.IPL3 = 0;    //currPosCntU interrupt priority level:<7
    INTCON1bits.NSTDIS = 0; //Allow interrupt nesting
    INTCON2bits.ALTIVT = 0; //use primary vector table

    /* ピン変化割り込み */
    CNEN1bits.CN2IE = enable;
    CNEN1bits.CN3IE = enable;
    IEC1bits.CNIE = enable;
    IFS1bits.CNIF = 0;

    /*割り込み許可ビット*/
    //IEC0bits.OC1IE = enable;
    IEC0bits.U1TXIE = disable; //UART1 tx interrupt enable
    IEC1bits.U2TXIE = disable;
    IEC0bits.U1RXIE = enable;
    IEC0bits.T1IE = enable; //timer1 interrupt enable
    IEC3bits.QEI1IE = enable;
    IEC4bits.QEI2IE = enable;

    /*割り込み優先度*/
    //IPC3bits.U1TXIP = 0x7; //interrupt is priority 6
    IPC7bits.U2TXIP = 0x7; //Interrupt priority level:6
    IPC2bits.U1RXIP = 0x6; //Interrupt priority level:6
    IPC14bits.QEI1IP = 0x5;
    IPC18bits.QEI2IP = 0x5;
    IPC0bits.T1IP = 0x5; //set interrupt priority level
#endif
    /*pin*/
    TRISBbits.TRISB2 = 0; //debug1
    TRISBbits.TRISB3 = 0; //debug2

    TRISBbits.TRISB15 = 0;     //UART2 transmission
    RPOR7bits.RP15R = 0b00101; //RP15-RB15: UART2 transmission

    TRISBbits.TRISB11 = 1;   //reception
    RPINR18bits.U1RXR = 0xB; //RP11 - UART Reception

    TRISBbits.TRISB7 = 1;    //QEI1A
    RPINR14bits.QEA1R = 0x7; //RP11
    TRISBbits.TRISB8 = 1;    //QEI1B
    RPINR14bits.QEB1R = 0x8; //RP10

    TRISBbits.TRISB10 = 1; //QEI2
    RPINR16bits.QEA2R = 0xA;
    TRISBbits.TRISB9 = 1;
    RPINR16bits.QEB2R = 0x9;

    TRISBbits.TRISB12 = 0; //motor1-front
    TRISAbits.TRISA2 = 0;  //motor1-back

    TRISAbits.TRISA3 = 0;  //motor2-front
    TRISBbits.TRISB13 = 0; //motor2-back

    TRISBbits.TRISB14 = 0;     //OC1 -RB14
    RPOR7bits.RP14R = 0b10010; //OC1 -RB14

    TRISBbits.TRISB4 = 0; //OC2-RB4
    RPOR2bits.RP4R = 0b10011;

    AD1PCFGL = 0xFFFF;

    /*UART1*/
    //PMD1bits.U1MD = disable; //U1 disable:disable
    U1MODE = 0x0808;
    U2STA = 0x2000;
    IFS0bits.U1TXIF = disable; //Interrupt flag disable
    IFS0bits.U1RXIF = disable; //Interrupt flag disable

    /*UART1 is enabled, continue module operation in idle mode,IrDA disable, U1RTS pin in simplex mode,
     * TX&RX pin are enabled and used, disable loopback, auto baud rate disable, high baud rate enable,receive idle status:1*/
    U1BRG = 165;
    U1MODEbits.UARTEN = enable; //UART1 is enabled
    U1STAbits.UTXEN = enable;   //transmit enable

    /*UART2*/
    //PMD1bits.U2MD = disable; //U1 disable:disable
    U2MODE = 0x0808;
    U2STA = 0x2000;
    IFS1bits.U2TXIF = disable; //Interrupt flag disable
    U2BRG = 165;
    U2MODEbits.UARTEN = enable; //UART1 is enabled
    U2STAbits.UTXEN = enable;   //transmit enable

    /*QEI1*/
    QEI1CONbits.QEISIDL = disable; //continue module operation in idle mode
    QEI1CONbits.QEIM = 0x7;        //QEI enabled (x4 mode) with position counter reset by match(MAX1CNT)
    QEI1CONbits.SWPAB = disable;   //A and B inputs not swapped-交換
    QEI1CONbits.PCDOUT = disable;  //direction status output disabled
    QEI1CONbits.TQGATE = disable;
    QEI1CONbits.UPDN = 1; //position counter direction is Positive
    MAX1CNT = 0xFFFF;
    POS1CNT = QEI_INIT_VALUE; /* カウントリセット */

    /*QEI2*/
    QEI2CONbits.QEISIDL = disable; //continue module operation in idle mode
    QEI2CONbits.QEIM = 0x7;        //QEI enabled (x4 mode) with position counter reset by match(MAX1CNT)
    QEI2CONbits.SWPAB = disable;   //A and B inputs not swapped-交換
    QEI2CONbits.PCDOUT = disable;  //direction status output disabled
    QEI2CONbits.TQGATE = disable;
    QEI2CONbits.UPDN = 1; //position counter direction is Positive
    MAX2CNT = 0xFFFF;
    POS2CNT = QEI_INIT_VALUE; /* カウントリセット */

    /*Timer1*/
    T1CONbits.TON = ON;                                    //Timer on
    T1CONbits.TCS = T1CONbits.TGATE = T1CONbits.TSYNC = 0; //Timer mode
    T1CONbits.TSIDL = 0;                                   //Continue module operation in Idle mode
    T1CONbits.TCKPS = 0x00;                                //prescale 1:1
    TMR1 = 0x0;
    PR1 = PID_TIMER_1MS;

    /*Timer2(for OC)*/
    T2CONbits.TCS = T2CONbits.TGATE = T2CONbits.T32 = OFF; //Mode:16bit Timer
    T2CONbits.TON = ON;
    T2CONbits.TCKPS = 0x00; /*プリスケーラ1：1*/

    /*OC1*/
    OC1CONbits.OCM = 0x6;  //Mode of OC1:PWM Mode.Fault pin disabled
    OC1CONbits.OCSIDL = 1; //halts in currPosCntU idle mode
    OC1CONbits.OCTSEL = 0; //timer2
    PR2 = OC_PERIOD;       //decide the length of period:144ms

    /*OC2*/
    OC2CONbits.OCM = 0x6;  //Mode of OC2:PWM Mode.Fault pin disabled
    OC2CONbits.OCSIDL = 1; //halts in currPosCntU idle mode
    OC2CONbits.OCTSEL = 0; //timer2
    PR2 = OC_PERIOD;       //decide the length of period:144/2ms

    /* モータを停止 */
    SetMotorIO(ALL, OFF);
}
