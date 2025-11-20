
#ifndef FUNCIONES_GENERALES_H_
#define FUNCIONES_GENERALES_H_

// Includes
#include <stdint.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "UART.h"
// Constantes

#define A 16000.0 / (5.0*1000.0)
// #define C  4.8828125 // C = 5000mV / 1024;
#define C  4.88 // C = 5000mV / 1024;

// Funciones de configuracion
void setup_ADC(void);
void setup_PWM(void);
void setup_SWITCHS(void);

// Funciones generales
uint16_t leer_ADC(void);
uint16_t tension_a_WC(uint16_t tension);


#endif /* FUNCIONES_GENERALES_H_ */