
#ifndef FUNCIONES_IDENTIFICACION_H_
#define FUNCIONES_IDENTIFICACION_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "UART.h"
#include "funciones_generales.h"

extern volatile uint16_t registro_LFSR;
extern volatile uint16_t N;
extern volatile uint8_t contador_PRBS;
extern volatile uint8_t flag_test_PRBS;

// Funciones de test
void comenzar_test_escalon(uint16_t bajo, uint16_t alto);
void comenzar_test_PRBS(void);
void terminar_test_PRBS(void);

// funciones de setup
void setup_LFSR(void);

// funciones secundarias de LFSR y PRBS
uint8_t LFSR_shift(void);
void actualizar_PWM_PRBS(void);

#endif /* FUNCIONES_IDENTIFICACION_H_ */