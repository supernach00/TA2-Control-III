
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
volatile int32_t u = 0;
volatile int32_t u_salida = 0;
const int32_t escalado = 1000;


// Coeficientes del PID
#define A0    1189
#define A1   -2132
#define A2    954
#define B1    1749
#define B2    -749

// Coeficientes del MPC
 #define kr    1193 
 #define ku    -977
 #define k1    -60
 #define k2    -55
 #define k3    -46
 #define k4    -32
 #define k5    -17
 #define k6    -5

//#define kr  229
//#define ku  -192
//#define k1   -11
//#define k2    -10
//#define k3    -8
//#define k4    -5
//#define k5    -3
//#define k6    -1


void aplicar_control_MPC(uint16_t ref){

	// Esta funcion los estados, aplica el controlador MPC y actualiza el valor del comparador OCR1A para ajustar el ciclo de trabajo de la PWM.

	leer_adc_multiple();
	uint16_t tension_control = controlador_MPC(ref);
	OCR1A = tension_a_WC(tension_control);
	
}

void aplicar_control_PID(uint16_t ref){

	// Esta funcion lee el ADC, aplica el controlador PID y actualiza el valor del comparador OCR1A para ajustar el ciclo de trabajo de la PWM.

	uint16_t entrada = leer_ADC();
	uint16_t tension_control = controlador_PID(ref, entrada);
	OCR1A = tension_a_WC(tension_control);
	
}

uint16_t controlador_MPC(uint16_t ref1){

	// Usamos 64 bits para evitar overflow durante la acumulacion
	int64_t acc =
		(int64_t)kr * ref1       +
		(int64_t)ku * u_salida   +
		(int64_t)k1 * estados[0] +
		(int64_t)k2 * estados[1] +
		(int64_t)k3 * estados[2] +
		(int64_t)k4 * estados[3] +
		(int64_t)k5 * estados[4] +
		(int64_t)k6 * estados[5];

	// División por el factor de escala
	int64_t deltaU = (acc / escalado); 

	u = u + deltaU;

	u_salida = u + delta;
	
	if (u < 0) u = 0;
	if (u > 5000) u = 5000;

	if (u_salida < 0) u_salida = 0;
	if (u_salida > 5000) u_salida = 5000;

	return u_salida;
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

	// Saturar la salida a rango válido de tensión en mV (0 - 5000)
	if (u < 0) u = 0;
	if (u > 5000) u = 5000;

	e2 = e1;
	e1 = e;

	u2 = u1;
	u1 = u;
	u = u - delta;

	return (uint16_t)u;
}

void cambiar_referencia(uint16_t bajo, uint16_t alto){

	referencia = (referencia == bajo) ? alto : bajo;

}