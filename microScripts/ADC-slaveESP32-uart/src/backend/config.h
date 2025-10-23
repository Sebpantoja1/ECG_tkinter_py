#ifndef CONFIG_H
#define CONFIG_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "string.h"

// ============ CONFIGURACIONES DEL SISTEMA ============
#define SAMPLE_RATE_HZ 2000
#define UART_PORT UART_NUM_0
#define UART_BAUD_RATE 115200
#define BUF_SIZE 1024
#define RD_BUF_SIZE 256
#define ADC_QUEUE_SIZE 200

// ============ PINES GPIO ============
#define ADC_CHANNEL ADC_CHANNEL_0
#define MUX_PIN_A GPIO_NUM_25
#define MUX_PIN_B GPIO_NUM_26
#define LED_PIN GPIO_NUM_2

// ============ CONFIGURACIONES ADC ============
#define ADC_UNIT_USED ADC_UNIT_1
#define ADC_BITWIDTH_USED ADC_BITWIDTH_12
#define ADC_ATTEN_USED ADC_ATTEN_DB_11

// ============ CONFIGURACIONES UART ============
#define UART_DATA_BITS_USED UART_DATA_8_BITS
#define UART_PARITY_USED UART_PARITY_DISABLE
#define UART_STOP_BITS_USED UART_STOP_BITS_1
#define UART_FLOW_CTRL_USED UART_HW_FLOWCTRL_DISABLE

// ============ PRIORIDADES DE TAREAS ============
#define ADC_TASK_PRIORITY 6
#define UART_TASK_PRIORITY 5
#define MUX_TASK_PRIORITY 4

// ============ TAMAÑOS DE STACK ============
#define ADC_STACK_SIZE 4096
#define UART_STACK_SIZE 4096
#define MUX_STACK_SIZE 2048

// ============ TIMEOUTS ============
#define MUTEX_TIMEOUT_MS 100
#define QUEUE_TIMEOUT_MS 1
#define UART_CMD_TIMEOUT_MS 10

// ============ PROTOCOLO DE DATOS ============
#define DATA_START_BYTE 0xAA
#define DATA_PACKET_SIZE 4

// ============ MULTIPLEXOR ============
#define MUX_DEFAULT_DELAY_MS 1000 
#define MUX_MIN_DELAY_MS 100
#define MUX_MAX_DELAY_MS 10000

// ============ LOGGING ============
static const char *TAG = "ECG_SYSTEM";

// ============ ENUMERACIONES ============
typedef enum {
    MUX_STATE_00 = 0,  // A(0) B(0) - Lead I
    MUX_STATE_01 = 1,  // A(0) B(1) - Lead II
    MUX_STATE_10 = 2,  // A(1) B(0) - Lead III
    MUX_STATE_11 = 3,  // A(1) B(1) - aVF
    MUX_STATE_STOP = 4 // Estado para parar
} mux_state_t;

// ============ ESTRUCTURAS ============
typedef struct {
    mux_state_t current_state;
    bool running;
    bool manual_mode;
    uint32_t delay_ms;
    bool state_changed;  // Flag para indicar cambio de estado
} mux_control_t;

typedef struct {
    int raw_value;
    uint32_t timestamp;
} adc_data_t;

typedef struct {
    char command[32];
    uint32_t timestamp;
} uart_command_t;

// ============ HANDLES GLOBALES ============
extern QueueHandle_t adc_data_queue;
extern QueueHandle_t uart_command_queue;
extern SemaphoreHandle_t mux_control_mutex;
extern adc_oneshot_unit_handle_t adc_handle;
extern esp_timer_handle_t adc_timer;

// ============ VARIABLES GLOBALES ============
extern mux_control_t mux_control;

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
void IRAM_ATTR uart_event_task(void* pvParameters);

// Control del multiplexor
void set_mux_state(mux_state_t state);
void process_mux_command(const char* command);

// Utilidades
void led_blink(void);
void send_data_packet(int adc_value);
void parse_uart_data(uint8_t* data, int length);

#endif // CONFIG_H