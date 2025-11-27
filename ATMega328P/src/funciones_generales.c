#include "funciones_generales.h"

void setup_ADC(void){
	
	// Esta funcion configura el ADC para realizar lecturas en el pin ADC5 con una referencia de 5V y un prescaler de 128.
	
	/*
	TIMER = ADC
	MODO = SINGLE CONVERSION
	PRESCALER = 128 (f_adc = 125Khz)
	ADC = ENABLE
	ADC INTERRUPT = ENABLE
	PIN DE ENTRADA = ADC5, A5
	REFERENCIA = 5V
	*/
	
	ADMUX = (1 << REFS0) | (1 << MUX0) | (1 << MUX2);
	ADCSRA = (1 << ADEN) | (0 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 	
	
	
	/*
	TIMER = 2
	MODO = CTC
	PRESCALER = 64 (f = 250kHz)
	COMPARADOR PERIODO = OCR2A = 249 (Interrupción cada 1 ms)
	*/
	
	// Modo CTC
    TCCR2A = (1 << WGM21);

    // Prescaler = 64
    TCCR2B = (1 << CS22);

    // Queremos 1 ms
    OCR2A = 249;

    // Habilitar interrupción por compare match
    TIMSK2 = (1 << OCIE2A);

    // Reset del contador
    TCNT2 = 0;
	
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
	//TIMSK1 |= (1 << TOIE1);
	
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
		
uint16_t tension_a_WC(uint16_t tension){
	
	// Esta funcion convierte un valor de tension en milivoltios a un entero entre 0 y 15999, que se utiliza
	// como valor de comparacion del timer1.
	// Esto define el ciclo de trabajo (WC) de la PWM de salida.
	
	return tension *  A - 1; //  A = 16000 / (5*1000)
	
	}
