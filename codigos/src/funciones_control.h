
#ifndef FUNCIONES_CONTROL_H_
#define FUNCIONES_CONTROL_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "UART.h"
#include "funciones_generales.h"

// Variables control
extern volatile int32_t e1;
extern volatile int32_t e2;
extern volatile int32_t u1;
extern volatile int32_t u2;
extern volatile uint16_t tension_entrada;
extern volatile uint16_t tension_filtrada;
extern volatile uint8_t perturbacion_activada;	
extern volatile uint16_t delta;
extern volatile uint16_t referencia;

// Funciones de control y filtros
uint16_t controlador_PID(uint16_t referencia, uint16_t medicion);
void aplicar_control_PID(uint16_t referencia);
void cambiar_referencia(uint16_t bajo, uint16_t alto);

#endif /* FUNCIONES_CONTROL_H_ */