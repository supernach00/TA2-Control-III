/*
 * TA3 Control III.c
 *
 * Created: 1/12/2025 
 * Author : nacho
 */ 

#include "UART.h"
#include "funciones_generales.h"
#include "funciones_identificacion.h"
#include "funciones_control.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

// Variables interrupciones
volatile uint8_t flag_muestreo = 0;
volatile uint8_t flag_perturbacion = 0;
volatile uint8_t contador_muestreo = 0;
volatile uint16_t contador_referencia = 0;
volatile uint16_t contador_perturbacion = 0;

ISR(TIMER2_COMPA_vect) // Interrupción cada 1 ms
{
    if (++contador_muestreo >= 30) // Muestreo
    {
        contador_muestreo = 0;
		flag_muestreo = 1;
		// PORTB ^= (1 << PB0);

	}

    if (++contador_referencia >= 10000) 
    {
        contador_referencia = 0;
		contador_perturbacion = 0;
		flag_perturbacion = 1;
		cambiar_referencia(1000, 4000); // Cambia la referencia entre 1V y 4V cada 10 segundos
		delta = (referencia == 1000) ? 0 : 1000;
		PORTB ^= (1 << PB0);
	}

	if (flag_perturbacion)
	{
		if (++contador_perturbacion >= 5000)  // Cada 5 segundos, se activa o desactiva la perturbación.
		{
			flag_perturbacion = 0;
			delta = (delta == 0) ? 1000 : 0; // Cambia entre 0 y 1V de perturbación.

		}
	}
}

ISR(PCINT2_vect) { // Interrupción cuando se presiona el switch 1 (PD4)
	
    if (PIND & (1 << PD4)) {

		perturbacion_activada = 1;
		delta = 3500; 
		PORTB |= (1 << PB0); // Enciende LED indicador
}
}

ISR(INT1_vect) // Interrupción cuando se presiona el switch 2 (PD3)
{

		perturbacion_activada = 0;
		delta = 0; 
		PORTB &= ~(1 << PB0); // Apaga LED indicador
}

ISR(TIMER0_COMPA_vect) // Código que se ejecuta a 61 Hz (cada 16.39 ms)
{

	// contador_PRBS++;

	// if (contador_PRBS >= 4) { 

	// 	// Cada 4 interrupciones de Timer0 (osea, cada 65.56 ms), se generan nuevos bits del PRBS.

	// 	N++; // N lleva cuenta de la cantidad de bits generados en el test de PRBS. Termina cuando N = 2047.

	// 	if (N == 2047) {

	// 		terminar_test_PRBS();

	// 	} else {
		
	// 		contador_PRBS = 0;
	// 		actualizar_PWM_PRBS();

	// 	}
	// }
}

	int main(void)
	{
		
		setup_ADC();
		setup_PWM();
		setup_SWITCHS();
		setup_LFSR();
		USART_init();
		sei();
		
		while (1)
		{

			if (flag_muestreo) {

				flag_muestreo = 0;
				aplicar_control_MPC(referencia);
			}
		}

		return 0;
	}

