/*
 * TA2 Control III.c
 *
 * Created: 18/11/2025 
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
volatile uint8_t flag_lectura_ADC = 0;
volatile uint8_t contador_30ms = 0;
volatile uint16_t contador_15seg = 0;

ISR(TIMER2_COMPA_vect) // Interrupción cada 1 ms
{
    if (++contador_30ms >= 30)
    {
        contador_30ms = 0;
		flag_lectura_ADC = 1;

	}

    if (++contador_15seg >= 15000) 
    {

        contador_15seg = 0;
		cambiar_referencia(1000, 4000); // Cambia la referencia entre 1V y 4V cada 10 segundos

	}
}

ISR(PCINT2_vect) { // Interrupción cuando se presiona el switch 1 (PD4)
	
    if (PIND & (1 << PD4)) {

		perturbacion_activada = 1;
		delta = 1000; 

    } else {

    }

};

ISR(INT1_vect) // Interrupción cuando se presiona el switch 2 (PD3)
{

		perturbacion_activada = 0;
		delta = 0; 
}

ISR(TIMER0_COMPA_vect) // Código que se ejecuta a 61 Hz (cada 16.39 ms)
{

	contador_PRBS++;

	if (contador_PRBS >= 4) { 

		// Cada 4 interrupciones de Timer0 (osea, cada 65.56 ms), se generan nuevos bits del PRBS.

		N++; // N lleva cuenta de la cantidad de bits generados en el test de PRBS. Termina cuando N = 2047.

		if (N == 2047) {

			terminar_test_PRBS();

		} else {
		
			contador_PRBS = 0;
			actualizar_PWM_PRBS();

		}
	}
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

			if (flag_lectura_ADC) {

				flag_lectura_ADC = 0;
				PORTB |= (1 << PB0);
				aplicar_control_PID(referencia);
				PORTB ^= (1 << PB0);
			}
		}

		return 0;
	}

