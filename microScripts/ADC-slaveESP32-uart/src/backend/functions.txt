#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "config.h"

// ============ DECLARACIONES DE FUNCIONES ============

// Inicialización
void system_init(void);
void adc_init(void);
void uart_init(void);
void gpio_init(void);
void timers_init(void);
void queues_init(void);
void mutexes_init(void);

// Tareas
void adc_acquisition_task(void* pvParameters);
void uart_communication_task(void* pvParameters);
void mux_control_task(void* pvParameters);

// Callbacks y funciones de interrupción
static void IRAM_ATTR adc_timer_callback(void* arg);

// Control del multiplexor
void set_mux_state(mux_state_t state);
void process_mux_command(const char* command);

// Utilidades
void led_blink(void);
void send_data_packet(int adc_value);
void parse_uart_data(uint8_t* data, int length);

#endif // FUNCTIONS_H