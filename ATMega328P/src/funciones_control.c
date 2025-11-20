
#include "funciones_control.h"

// Variables control
volatile uint16_t tension_entrada = 0;
volatile uint16_t tension_filtrada = 0;
volatile uint16_t referencia = 1000; // (En milivoltios)
volatile uint8_t perturbacion_activada = 0;
volatile uint16_t delta = 0;
volatile int32_t e1 = 0;
volatile int32_t e2 = 0;
volatile int32_t u1 = 0;
volatile int32_t u2 = 0;
const int32_t escalado = 1000;

// Coeficientes del PID
#define A0    1189
#define A1   -2132
#define A2    954
#define B1    1749
#define B2    -749

void aplicar_control_PID(uint16_t ref){

	// Esta funcion lee el ADC, aplica el controlador PID y actualiza el valor del comparador OCR1A para ajustar el ciclo de trabajo de la PWM.

	uint16_t entrada = leer_ADC();
	uint16_t tension_control = controlador_PID(ref, entrada);
	OCR1A = tension_a_WC(tension_control);
	
}

uint16_t controlador_PID(uint16_t ref1, uint16_t medicion){

	int32_t e = (int32_t)ref1 - (int32_t)medicion;

	// Usamos 64 bits para evitar overflow durante la acumulacion
	int64_t acc =
		  (int64_t)A0 * e
		+ (int64_t)A1 * e1
		+ (int64_t)A2 * e2
		+ (int64_t)B1 * u1
		+ (int64_t)B2 * u2;

	// División por el factor de escala
	int64_t u = (acc / escalado); 

	// Saturar la salida a rango válido de tensión en mV (0 .. 5000)
	if (u < 0) u = 0;
	if (u > 5000) u = 5000;

	e2 = e1;
	e1 = e;

	u2 = u1;
	u1 = u;
	u = u + delta;

	return (uint16_t)u;
}

void cambiar_referencia(uint16_t bajo, uint16_t alto){

	referencia = (referencia == bajo) ? alto : bajo;

}