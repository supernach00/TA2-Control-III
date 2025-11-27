
#include "funciones_identificacion.h"

volatile uint16_t registro_LFSR;
volatile uint16_t N;
volatile uint8_t contador_PRBS = 0;
volatile uint8_t flag_test_PRBS = 0;

void comenzar_test_escalon(uint16_t bajo, uint16_t alto){

	// Esta funcion genera un escalon con una pwm en el pin PB1, que va desde un valor "bajo (mV)" hasta un valor "alto (mV)".
	// Luego se repite el test pero con  un escalon en sentido contrario.
	// En el pin PB2 genera una señal sincronizada al escalón, pero entre 0V y 1V, que se utiliza para medir la señal de entrada.

	DDRB |= (1 << PB2); // PB2 como salida

	// Escalon de bajo a alto
	OCR1A = tension_a_WC(bajo);
	_delay_ms(3000);

	OCR1A = tension_a_WC(alto);
	PORTB |= (1 << PB2); // Señal de sincronización a 1V
	_delay_ms(3000);

	// Escalon de alto a bajo
	OCR1A = tension_a_WC(bajo);
	PORTB &= ~(1 << PB2); // Señal de sincronización a 0V
	_delay_ms(3000);

	// Apago salida
	OCR1A = 0;
	
	}

void setup_LFSR(void){

	// Esta funcion configura el Timer0 para generar interrupciones a aproximadamente 61 Hz, que se utilizan para actualizar la señal PRBS.
	// También configura el pin PB2 como salida para la señal de sincronización.

	DDRB |= (1 << PB2); // PB2 como salida para la señal sincronizada

	/*
	TIMER = 0
	MODO = CTC
	PRESCALER = 1024 (f = 15625Hz)
	COMPARADOR PERIODO = OCR0A = 255
	*/

	TCCR0A = (1 << WGM01);              // Modo CTC (WGM01=1, WGM00=0)
    TCCR0B = (1 << CS02) | (1 << CS00); // Prescaler = 1024 (CS02=1, CS00=1)

    OCR0A = 255;  // valor máximo posible, frecuencia mínima de 61 Hz

	}


uint8_t LFSR_shift(void) {

	// Esta funcion realiza un corrimiento del registro LFSR de 11 bits y devuelve el bit de salida (0 o 1).
	// La ralimentación se realiza en el bit 1 con el xor de los bits 9 y 11.

    uint8_t bit9 = (registro_LFSR >> 8) & 1;
    uint8_t bit11 = (registro_LFSR >> 10) & 1;
    uint8_t feedback_bit = bit9 ^ bit11;

    uint8_t bit_salida = registro_LFSR & 1;

	registro_LFSR = (registro_LFSR << 1) | (feedback_bit);

	return bit_salida;

}


void actualizar_PWM_PRBS(void){

	// Esta funcion convierte el valor del bit generado por el LFSR en una señal PWM.
	// El bit de salida del LFSR se utiliza para definir dos niveles de tensión:
	// 1000mV (0) y 4000mV (1) con la PWM.
	// En el pin PB2 genera una señal sincronizada a la PRBS, pero entre 0V y 5V, que se utiliza para medir la señal de entrada.

	uint8_t bit_LFSR = LFSR_shift();

	if (bit_LFSR == 1){
		
		OCR1A = tension_a_WC(4000);
		PORTB |= (1 << PB2); // Señal de sincronización a 5V

		} 

	else {

		OCR1A = tension_a_WC(1000);
		PORTB &= ~(1 << PB2); // Señal de sincronización a 0V

		}
	}

void comenzar_test_PRBS(void){

	// Esta funcion inicia el test PRBS: reinicia contadores, define el valor semilla del LFSR y habilita la interrupción del Timer0.

	terminar_test_PRBS(); // Asegura que el test PRBS esté detenido antes de comenzar uno nuevo

	N = 0; // Contador de las iteraciones del PRBS, es decir, la cantidad de bits generados hasta el momento.

	registro_LFSR = 0b01100110011; // Estado inicial del LFSR de 11 bits.

    TIMSK0 = (1 << OCIE0A);  // Habilitar interrupción por comparación con OCR0A

	}

void terminar_test_PRBS(void){

	// Esta funcion detiene el test PRBS. Deshabilita la interrupción y apaga la salida PWM y la señal de sincronización.

	TIMSK0 &= ~(1 << OCIE0A);  // Deshabilitar interrupción por comparación con OCR0A

	OCR1A = 0; // Apagar salida PWM
	PORTB &= ~(1 << PB2); // Señal de sincronización a 0V

}
