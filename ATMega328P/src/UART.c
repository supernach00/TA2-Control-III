#include <avr/io.h>
#include <stdio.h>
#include "UART.h"

void USART_init(void) {
	// Configurar Baud Rate
	UBRR0H = (uint8_t)(BAUD_PRESCALLER >> 8);
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);

	// Habilitar transmisi�n y recepci�n
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);

	// Configuraci�n: 8 bits de datos, 1 bit de stop, sin paridad
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void USART_send(unsigned char data) {
	// Esperar hasta que el buffer de transmisi�n est� vac�o
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

unsigned char USART_receive(void) {
	// Esperar hasta que llegue un dato
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

void USART_putstring(char* StringPtr) {
	while (*StringPtr) {
		USART_send(*StringPtr++);
	}
}

void USART_put_uint16(uint16_t variable) {
	char string_out[10];
	sprintf(string_out, "%d", variable);
	USART_putstring(string_out);
	USART_putstring("\r\n");
	_delay_ms(500);
	
}

