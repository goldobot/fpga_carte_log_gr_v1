#ifndef __ROBOT_UART_H
#define __ROBOT_UART_H

enum uart_baudrate_t { B9600, B19200, B38400, B57600, B115200, BAUTOBAUD };

enum uart_parity_t { PARITY_NONE, PARITY_EVEN, PARITY_ODD };

/* 0: Receiver enable (RE) - if set, enables the receiver. */
#define UART_CONTROL_RE		0x00000001	
/* 1: Transmitter enable (TE) - if set, enables the transmitter. */
#define UART_CONTROL_TE		0x00000002	
/* 2: Receiver interrupt enable (RI) - if set, enables generation of
   receiver interrupt. */
#define UART_CONTROL_RI		0x00000004	
/* 3: Transmitter interrupt enable (TI) - if set, enables generation
   of transmitter interrupt */
#define UART_CONTROL_TI		0x00000008	
/* 4: Parity select (PS) - selects parity polarity (0 = even parity, 1
   = odd parity) */
#define UART_CONTROL_PS		0x00000010	
/* 5: Parity enable (PE) - if set, enables parity generation and
   checking. */
#define UART_CONTROL_PE		0x00000020	
/* 6: Flow control (FL) - if set, enables flow control using
   CTS/RTS. */
#define UART_CONTROL_FL		0x00000040	
/* 7: Loop back (LB) - if set, loop back mode will be enabled. */
#define UART_CONTROL_LB		0x00000080	
/* 8: External Clock (EC) - if set, the UART use clock generated by
   autobaud block */
#define UART_CONTROL_EC		0x00000100	

/* 0: Data ready (DR) - indicates that new data is available in the
   receiver holding register. */
#define UART_STATUS_DR		0x00000001	
/* 1: Transmitter shift register empty (TS) - indicates that the
   transmitter shift register is empty. */
#define UART_STATUS_TS		0x00000002	
/* 2: Transmitter hold register empty (TH) - indicates that the
   transmitter hold register is empty. */
#define UART_STATUS_TH		0x00000004	
/* 3: Break received (BR) - indicates that a BREAK has been received. */
#define UART_STATUS_BR		0x00000008	
/* 4: Overrun (OV) - indicates that one or more character have been
   lost due to overrun. */
#define UART_STATUS_OV		0x00000010	
/* 5: Parity error (PE) - indicates that a parity error was
   detected. */
#define UART_STATUS_PE		0x00000020	
/* 6: Framing error (FE) - indicates that a framing error was detected. */
#define UART_STATUS_FE		0x00000040	
/* 7: Autobaud Locked (AL) - indicates that the autobaud block as sent
   a lock signal. */
#define UART_STATUS_AL		0x00000080	

#include "types.h"

/* get scaler currently associated to given baudrate

   @param[in] bd = baudrate

   @return current scaler value */
uint32_t uart_get_scaler ( enum uart_baudrate_t bd );

/* set scaler for the given baudrate

   @param[in] bd = baudrate
   @param[in] scaler = new scaler value */
void uart_set_scaler ( enum uart_baudrate_t bd, uint32_t scaler );

/* calibrate autobaud, i.e. compute scaler value for BAUTOBAUD baudrate */
void uart_calibrate ();

/* get current parity */
enum uart_parity_t uart_get_parity ();

/* set parity

   @param[in] parity = parity */
void uart_set_parity ( enum uart_parity_t parity );

/* enable UART at given baudrate */
void uart_init ( enum uart_baudrate_t bd );

/* send byte on UART (UART must be enabled before)

   @param[in] byte = byte to send */
void uart_putchar ( uint8_t byte );

/* flush output buffer */
void uart_flush ();

/* wait for a byte on UART and get it (UART must be enabled before)

 @return byte read on UART */
uint8_t uart_getchar ();

/* non blocking version of uart_getchar (UART must be enabled before)

   WARNING: timer1 is used to implement the timeout

   @param[in] time_ms = timeout in ms
   @param[out] byte = address where to write read byte if any

   @return 1 if a byte was available in given time, 0 otherwise */
int uart_try_getchar ( unsigned int time_ms, uint8_t *byte );

int uart_getchar_in_fifo ( uint8_t *byte );

void uart_putstring ( uint8_t *s );

void uart_printhex ( uint32_t val );

void uart_printint ( int val );

#endif
