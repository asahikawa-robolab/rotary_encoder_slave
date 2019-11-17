/*-----------------------------------------------
 *
 * Last updated : 2020/02/02, 17:48
 *
-----------------------------------------------*/
#ifndef DSPIC33FJ128MC802_H
#define DSPIC33FJ128MC802_H

#include <xc.h>

/*-----------------------------------------------
 *
 * Timer
 *
-----------------------------------------------*/
#define T1ON T1CONbits.TON
#define T1CKPS T1CONbits.TCKPS
#define T2ON T2CONbits.TON
#define T2CKPS T2CONbits.TCKPS
#define T3ON T3CONbits.TON
#define T3CKPS T3CONbits.TCKPS
#define T4ON T4CONbits.TON
#define T4CKPS T4CONbits.TCKPS
#define T5ON T5CONbits.TON
#define T5CKPS T5CONbits.TCKPS

/*-----------------------------------------------
 *
 * OC
 *
-----------------------------------------------*/
#define OC1M OC1CONbits.OCM
#define OC1TSEL OC1CONbits.OCTSEL
#define OC2M OC2CONbits.OCM
#define OC2TSEL OC2CONbits.OCTSEL
#define OC3M OC3CONbits.OCM
#define OC3TSEL OC3CONbits.OCTSEL
#define OC4M OC4CONbits.OCM
#define OC4TSEL OC4CONbits.OCTSEL

/*-----------------------------------------------
 *
 * UART
 *
-----------------------------------------------*/
#define T1ON T1CONbits.TON
#define T1CKPS T1CONbits.TCKPS
#define T2ON T2CONbits.TON
#define T2CKPS T2CONbits.TCKPS
#define T3ON T3CONbits.TON
#define T3CKPS T3CONbits.TCKPS
#define T4ON T4CONbits.TON
#define T4CKPS T4CONbits.TCKPS
#define U1RXISEL U1STAbits.URXISEL
#define UART1EN U1MODEbits.UARTEN
#define U1TXEN U1STAbits.UTXEN
#define U1OERR U1STAbits.OERR
#define U1FERR U1STAbits.FERR
#define U1RXDA U1STAbits.URXDA
#define U2RXISEL U2STAbits.URXISEL
#define UART2EN U2MODEbits.UARTEN
#define U2TXEN U2STAbits.UTXEN
#define U2OERR U2STAbits.OERR
#define U2FERR U2STAbits.FERR
#define U2RXDA U2STAbits.URXDA

/*-----------------------------------------------
 *
 * SPI
 *
-----------------------------------------------*/
#define SPI1TBF SPI1STATbits.SPITBF
#define SPI1RBF SPI1STATbits.SPIRBF
#define SPI1ROV SPI1STATbits.SPIROV
#define SPI1EN SPI1STATbits.SPIEN
#define MSTEN_1 SPI1CON1bits.MSTEN
#define DISSDO_1 SPI1CON1bits.DISSDO
#define DISSCK_1 SPI1CON1bits.DISSCK
#define MODE16_1 SPI1CON1bits.MODE16
#define SMP_1 SPI1CON1bits.SMP
#define CKE_1 SPI1CON1bits.CKE
#define CKP_1 SPI1CON1bits.CKP
#define PPRE_1 SPI1CON1bits.PPRE
#define SPRE_1 SPI1CON1bits.SPRE
#define SSEN_1 SPI1CON1bits.SSEN
#define SPI2TBF SPI2STATbits.SPITBF
#define SPI2RBF SPI2STATbits.SPIRBF
#define SPI2ROV SPI2STATbits.SPIROV
#define SPI2EN SPI2STATbits.SPIEN
#define MSTEN_2 SPI2CON2bits.MSTEN
#define DISSDO_2 SPI2CON2bits.DISSDO
#define DISSCK_2 SPI2CON2bits.DISSCK
#define MODE26_2 SPI2CON2bits.MODE26
#define SMP_2 SPI2CON2bits.SMP
#define CKE_2 SPI2CON2bits.CKE
#define CKP_2 SPI2CON2bits.CKP
#define PPRE_2 SPI2CON2bits.PPRE
#define SPRE_2 SPI2CON2bits.SPRE
#define SSEN_2 SPI2CON2bits.SSEN

/*-----------------------------------------------
 *
 * I2C
 *
-----------------------------------------------*/
#define A10M_1 I2C1CONbits.A10M
#define ACKDT_1 I2C1CONbits.ACKDT
#define ACKEN_1 I2C1CONbits.ACKEN
#define DISSLW_1 I2C1CONbits.DISSLW
#define GCEN_1 I2C1CONbits.GCEN
#define I2CEN_1 I2C1CONbits.I2CEN
#define I2CSIDL_1 I2C1CONbits.I2CSIDL
#define IPMIEN_1 I2C1CONbits.IPMIEN
#define PEN_1 I2C1CONbits.PEN
#define RCEN_1 I2C1CONbits.RCEN
#define RSEN_1 I2C1CONbits.RSEN
#define SCLREL_1 I2C1CONbits.SCLREL
#define SEN_1 I2C1CONbits.SEN
#define SMEN_1 I2C1CONbits.SMEN
#define STREN_1 I2C1CONbits.STREN
#define ACKSTAT_1 I2C1STATbits.ACKSTAT
#define ADD10_1 I2C1STATbits.ADD10
#define BCL_1 I2C1STATbits.BCL
#define D_A_1 I2C1STATbits.D_A
#define GCSTAT_1 I2C1STATbits.GCSTAT
#define I2COV_1 I2C1STATbits.I2COV
#define IWCOL_1 I2C1STATbits.IWCOL
#define P_1 I2C1STATbits.P
#define RBF_1 I2C1STATbits.RBF
#define R_W_1 I2C1STATbits.R_W
#define S_1 I2C1STATbits.S
#define TBF_1 I2C1STATbits.TBF
#define TRSTAT_1 I2C1STATbits.TRSTAT

/*-----------------------------------------------
 *
 * QEI
 *
-----------------------------------------------*/
#define UPDN1 QEI1CONbits.UPDN
#define UPDN2 QEI2CONbits.UPDN

/*-----------------------------------------------
 *
 * 割り込み
 *
-----------------------------------------------*/
/* 割り込みフラグ */
#define U1TXIF IFS0bits.U1TXIF
#define U1RXIF IFS0bits.U1RXIF
#define U2TXIF IFS1bits.U2TXIF
#define U2RXIF IFS1bits.U2RXIF
#define SPI1IF IFS0bits.SPI1IF
#define SPI1EIF IFS0bits.SPI1EIF
#define SPI2IF IFS2bits.SPI2IF
#define SPI2EIF IFS2bits.SPI2EIF
#define INT0IF IFS0bits.INT0IF
#define QEI1IF IFS3bits.QEI1IF
#define QEI2IF IFS4bits.QEI2IF
#define T1IF IFS0bits.T1IF
#define T2IF IFS0bits.T2IF
#define T3IF IFS0bits.T3IF
#define T4IF IFS1bits.T4IF
#define T5IF IFS1bits.T5IF
#define CNIF IFS1bits.CNIF /* ピン変化割り込み */
/* 割り込み許可 */
#define U1TXIE IEC0bits.U1TXIE
#define U1RXIE IEC0bits.U1RXIE
#define U2TXIE IEC1bits.U2TXIE
#define U2RXIE IEC1bits.U2RXIE
#define SPI1IE IEC0bits.SPI1IE
#define SPI1EIE IEC0bits.SPI1EIE
#define SPI2IE IEC2bits.SPI2IE
#define SPI2EIE IEC2bits.SPI2EIE
#define INT0IE IEC0bits.INT0IE
#define QEI1IE IEC3bits.QEI1IE
#define QEI2IE IEC4bits.QEI2IE
#define T1IE IEC0bits.T1IE
#define T2IE IEC0bits.T2IE
#define T3IE IEC0bits.T3IE
#define T4IE IEC1bits.T4IE
#define T5IE IEC1bits.T5IE
#define CNIE IEC1bits.CNIE /* ピン変化割り込み */
/* 割り込み優先度 */
#define INT0IP IPC0bits.INT0IP
#define U1TXIP IPC3bits.U1TXIP
#define U1RXIP IPC2bits.U1RXIP
#define U2TXIP IPC7bits.U2TXIP
#define U2RXIP IPC7bits.U2RXIP
#define SPI1IP IPC2bits.SPI1IP
#define SPI1EIP IPC2bits.SPI1EIP
#define SPI2IP IPC8bits.SPI2IP
#define SPI2EIP IPC8bits.SPI2EIP
#define QEI1IP IPC14bits.QEI1IP
#define QEI2IP IPC18bits.QEI2IP
#define T1IP IPC0bits.T1IP
#define T2IP IPC1bits.T2IP
#define T3IP IPC2bits.T3IP
#define T4IP IPC6bits.T4IP
#define T5IP IPC7bits.T5IP
#define CNIP IPC4bits.CNIP /* ピン変化割り込み */

/*-----------------------------------------------
 *
 * PPS（ペリフェラルピンセレクト）
 *
-----------------------------------------------*/
#define RP0IN 0
#define RP1IN 1
#define RP2IN 2
#define RP3IN 3
#define RP4IN 4
#define RP5IN 5
#define RP6IN 6
#define RP7IN 7
#define RP8IN 8
#define RP9IN 9
#define RP10IN 10
#define RP11IN 11
#define RP12IN 12
#define RP13IN 13
#define RP14IN 14
#define RP15IN 15
#define RP0OUT RPOR0bits.RP0R
#define RP1OUT RPOR0bits.RP1R
#define RP2OUT RPOR1bits.RP2R
#define RP3OUT RPOR1bits.RP3R
#define RP4OUT RPOR2bits.RP4R
#define RP5OUT RPOR2bits.RP5R
#define RP6OUT RPOR3bits.RP6R
#define RP7OUT RPOR3bits.RP7R
#define RP8OUT RPOR4bits.RP8R
#define RP9OUT RPOR4bits.RP9R
#define RP10OUT RPOR5bits.RP10R
#define RP11OUT RPOR5bits.RP11R
#define RP12OUT RPOR6bits.RP12R
#define RP13OUT RPOR6bits.RP13R
#define RP14OUT RPOR7bits.RP14R
#define RP15OUT RPOR7bits.RP15R
#define U1TXR 3
#define U2TXR 5
#define SDO1R 7
#define SCKO1R 8
#define OC1R_ 18 /* 再定義の警告が出るためアンダーバーをつけて対応 */
#define OC2R_ 19
#define OC3R_ 20
#define OC4R_ 21
#define U1RXR RPINR18bits.U1RXR
#define U2RXR RPINR19bits.U2RXR
#define QEA1R RPINR14bits.QEA1R
#define QEB1R RPINR14bits.QEB1R
#define QEA2R RPINR16bits.QEA2R
#define QEB2R RPINR16bits.QEB2R
#define SDI1R RPINR20bits.SDI1R
#define SCKI1R RPINR20bits.SCK1R
#define SS1R RPINR21bits.SS1R

/*-----------------------------------------------
 *
 * IO
 *
-----------------------------------------------*/
#define TRISA0 TRISAbits.TRISA0
#define TRISA1 TRISAbits.TRISA1
#define TRISA2 TRISAbits.TRISA2
#define TRISA3 TRISAbits.TRISA3
#define TRISA4 TRISAbits.TRISA4
#define TRISB0 TRISBbits.TRISB0
#define TRISB1 TRISBbits.TRISB1
#define TRISB2 TRISBbits.TRISB2
#define TRISB3 TRISBbits.TRISB3
#define TRISB4 TRISBbits.TRISB4
#define TRISB5 TRISBbits.TRISB5
#define TRISB6 TRISBbits.TRISB6
#define TRISB7 TRISBbits.TRISB7
#define TRISB8 TRISBbits.TRISB8
#define TRISB9 TRISBbits.TRISB9
#define TRISB10 TRISBbits.TRISB10
#define TRISB11 TRISBbits.TRISB11
#define TRISB12 TRISBbits.TRISB12
#define TRISB13 TRISBbits.TRISB13
#define TRISB14 TRISBbits.TRISB14
#define TRISB15 TRISBbits.TRISB15
#define RA0 PORTAbits.RA0
#define RA1 PORTAbits.RA1
#define RA2 PORTAbits.RA2
#define RA3 PORTAbits.RA3
#define RA4 PORTAbits.RA4
#define RB0 PORTBbits.RB0
#define RB1 PORTBbits.RB1
#define RB2 PORTBbits.RB2
#define RB3 PORTBbits.RB3
#define RB4 PORTBbits.RB4
#define RB5 PORTBbits.RB5
#define RB6 PORTBbits.RB6
#define RB7 PORTBbits.RB7
#define RB8 PORTBbits.RB8
#define RB9 PORTBbits.RB9
#define RB10 PORTBbits.RB10
#define RB11 PORTBbits.RB11
#define RB12 PORTBbits.RB12
#define RB13 PORTBbits.RB13
#define RB14 PORTBbits.RB14
#define RB15 PORTBbits.RB15
#define LATA0 LATAbits.LATA0
#define LATA1 LATAbits.LATA1
#define LATA2 LATAbits.LATA2
#define LATA3 LATAbits.LATA3
#define LATA4 LATAbits.LATA4
#define LATB0 LATBbits.LATB0
#define LATB1 LATBbits.LATB1
#define LATB2 LATBbits.LATB2
#define LATB3 LATBbits.LATB3
#define LATB4 LATBbits.LATB4
#define LATB5 LATBbits.LATB5
#define LATB6 LATBbits.LATB6
#define LATB7 LATBbits.LATB7
#define LATB8 LATBbits.LATB8
#define LATB9 LATBbits.LATB9
#define LATB10 LATBbits.LATB10
#define LATB11 LATBbits.LATB11
#define LATB12 LATBbits.LATB12
#define LATB13 LATBbits.LATB13
#define LATB14 LATBbits.LATB14
#define LATB15 LATBbits.LATB15

/*-----------------------------------------------
 *
 * CN（ピン変化通知）
 *
-----------------------------------------------*/
#define CN0IE CNEN1bits.CN0IE
#define CN1IE CNEN1bits.CN1IE
#define CN2IE CNEN1bits.CN2IE
#define CN3IE CNEN1bits.CN3IE
#define CN4IE CNEN1bits.CN4IE
#define CN5IE CNEN1bits.CN5IE
#define CN6IE CNEN1bits.CN6IE
#define CN7IE CNEN1bits.CN7IE
#define CN11IE CNEN1bits.CN11IE
#define CN12IE CNEN1bits.CN12IE
#define CN13IE CNEN1bits.CN13IE
#define CN14IE CNEN1bits.CN14IE
#define CN15IE CNEN1bits.CN15IE
#define CN16IE CNEN2bits.CN16IE
#define CN21IE CNEN2bits.CN21IE
#define CN22IE CNEN2bits.CN22IE
#define CN23IE CNEN2bits.CN23IE
#define CN24IE CNEN2bits.CN24IE
#define CN27IE CNEN2bits.CN27IE
#define CN29IE CNEN2bits.CN29IE
#define CN30IE CNEN2bits.CN30IE

#endif