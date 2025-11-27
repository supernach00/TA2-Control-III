#ifndef UART_H_
#define UART_H_

#define F_CPU        16000000UL
#define BAUDRATE     9600
#define BAUD_PRESCALLER ((F_CPU / (16UL * BAUDRATE)) - 1)

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>


// Declaración de funciones
void USART_init(void);
unsigned char USART_receive(void);
void USART_send(unsigned char data);
void USART_putstring(char *StringPtr);
void USART_put_uint16(uint16_t variable);

#endif /* UART_H_ */
