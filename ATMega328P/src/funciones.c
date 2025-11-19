#include "funciones.h"
#include <avr/io.h>
#include <avr/interrupt.h>
extern volatile uint16_t tension_entrada;
extern volatile uint16_t tension_filtrada;
extern volatile uint16_t registro_LFSR;
extern volatile uint16_t N;

void setup_ADC(void){
	
	// Esta funcion configura el ADC para realizar lecturas en el pin ADC1 con una referencia de 5V y un prescaler de 128.
	
	/*
	PRESCALER = 128 (f_adc = 125Khz)
	ADC = ENABLE
	ADC INTERRUPT = ENABLE
	PIN DE ENTRADA = ADC1
	REFERENCIA = 5V
	*/
	
	ADMUX = (1 << REFS0) | (1 << MUX0);
	ADCSRA = (1 << ADEN) | (0 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 	
	
	}
	
void setup_SWITCHS(void){
	
	// Esta funcion configura el pin PD4 como entrada con resistencia pull-up y habilita la interrupción por cambio de pin.

	/*
	PIN DE ENTRADA = PD4
	CONFIGURACION = PULL-UP
	*/

	// Configurar PD4 como entrada con pull-up

	DDRD &= ~(1 << PD4);  
    PORTD |= (1 << PD4); 

    // Activar Pin Change Interrupt para Port D
    PCICR |= (1 << PCIE2);     // PCIE2 controla Port D
    PCMSK2 |= (1 << PCINT20);  // PD4 = PCINT20
	
	// Configurar interrupción externa INT1 en PD3

	EICRA |= (1 << ISC11);   // Configura interrupción INT1 en flanco descendente
	EIMSK |= (1 << INT1);    // Habilita la interrupción INT1

	}

void setup_PWM(void){
	
	// Esta funcion configura el Timer1 para generar una señal PWM en el pin PB1.
	
	/*
	TIMER = 1
	MODO = FAST PWM
	PRESCALER = 1 (f = 16MHz)
	COMPARADOR PERIODO = ICR1 = 16000 (1ms)
	COMPARADOR DC = OCR1A
	PIN DE  SALIDA = PB1
	DC INICIAL = 50%
	*/
	
	DDRB |= (1 << PB1); // OC1A = PB1 en Arduino Uno

	TCCR1A = (1 << WGM11) | (1 << COM1A1);
	TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10); // 
	TIMSK1 |= (1 << TOIE1);
	
	ICR1 = 15999;
	OCR1A = 1;  // Valor inicial de ciclo de trabajo.
	
	}
	
uint16_t leer_ADC(){
	
	// Esta funcion realiza una lectura del ADC y devuelve el valor en milivoltios.

	ADCSRA |= (1 << ADSC);
	
	while ( !(ADCSRA & (1 << ADIF)) ); // Se espera hasta que termine la conversion.

	ADCSRA |= (1 << ADIF);

	return ADC*C; // Devuelve el valor en milivoltios. C = 5.
	
	}
		
uint16_t filtro_RC(uint16_t error){
	
	// Esta funcion realiza un filtrado RC digital
	
	float tension_salida = (alfa * tension_filtrada + beta * error);
	
	return (uint16_t) tension_salida;
	
	}

uint16_t tension_a_WC(uint16_t tension){
	
	// Esta funcion convierte un valor de tension en milivoltios a un entero entre 0 y 15999, que se utiliza
	// como valor de comparacion del timer1.
	// Esto define el ciclo de trabajo (WC) de la PWM de salida.
	
	return tension *  A - 1; //  A = 16000 / (5*1000)
	
	}


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

void aplicar_filtro_RC(uint16_t referencia){

	// Esta funcion lee el ADC, aplica el filtro digital RC  y actualiza el valor del comparador OCR1A para ajustar el ciclo de trabajo de la PWM.

	USART_put_uint16(tension_entrada); 
	tension_entrada = leer_ADC();
	tension_filtrada = filtro_RC(referencia - tension_entrada);
	OCR1A = tension_a_WC(tension_filtrada);

}

void aplicar_control_PID(uint16_t referencia){

	// Esta funcion lee el ADC, aplica el controlador PID y actualiza el valor del comparador OCR1A para ajustar el ciclo de trabajo de la PWM.

	tension_entrada = leer_ADC();
	uint16_t tension_control = controlador_PID(referencia, tension_entrada);
	OCR1A = tension_a_WC(tension_control);
	
}

uint16_t controlador_PID(uint16_t referencia, uint16_t medicion){

}