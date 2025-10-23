#include "backend\config.h"
//#include "backend\functions.h"

void app_main(void) {
    // ============ INICIALIZACIÓN DEL SISTEMA ============
    system_init();
    
    // ============ CREAR TAREAS CON ASIGNACIÓN DE NÚCLEOS ============
    
    // Tarea de adquisición ADC - CORE 1 (Mayor prioridad)
    // Esta tarea maneja la recepción de datos del timer y envío por UART
    xTaskCreatePinnedToCore(
        adc_acquisition_task,
        "adc_acquisition",
        ADC_STACK_SIZE,
        NULL,
        ADC_TASK_PRIORITY,
        NULL,
        1  // Core 1 - Dedicado a tiempo real
    );
    
    // Tarea de comunicación UART - CORE 1 
    // Maneja los comandos entrantes y la comunicación
    xTaskCreatePinnedToCore(
        uart_communication_task,
        "uart_communication",
        UART_STACK_SIZE,
        NULL,
        UART_TASK_PRIORITY,
        NULL,
        1  // Core 1 - Junto con ADC para minimizar latencia
    );
    
    // Tarea de control del multiplexor - CORE 0
    // Control de estados del MUX y respuesta a comandos
    xTaskCreatePinnedToCore(
        mux_control_task,
        "mux_control",
        MUX_STACK_SIZE,
        NULL,
        MUX_TASK_PRIORITY,
        NULL,
        0  // Core 0 - Separado para evitar interferencias
    );
    
    // ============ MOSTRAR INFORMACIÓN DEL SISTEMA ============
    ESP_LOGI(TAG, "=== SISTEMA ECG INICIADO CORRECTAMENTE ===");
    ESP_LOGI(TAG, "Frecuencia de muestreo ADC: %d Hz", SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "Baudrate UART: %d", UART_BAUD_RATE);
    ESP_LOGI(TAG, "Tareas distribuidas en ambos núcleos");
    ESP_LOGI(TAG, "Filtrado digital deshabilitado (procesamiento en Python)");
    ESP_LOGI(TAG, "Sistema Lead-Off removido");
    
    // ============ COMANDOS DISPONIBLES ============
    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║          COMANDOS MUX DISPONIBLES        ║\n");
    printf("╠══════════════════════════════════════════╣\n");
    printf("║ START       - Iniciar modo automático   ║\n");
    printf("║ STOP        - Detener multiplexor       ║\n");
    printf("║ STATE_X     - Estado manual (X=0,1,2,3) ║\n");
    printf("║             0: Lead I   (A=0, B=0)       ║\n");
    printf("║             1: Lead II  (A=0, B=1)       ║\n");
    printf("║             2: Lead III (A=1, B=0)       ║\n");
    printf("║             3: aVF      (A=1, B=1)       ║\n");
    printf("║ DELAY_XXXX  - Delay en ms (100-60000)   ║\n");
    printf("║ STATUS      - Mostrar estado actual     ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");
    
    // ============ INFORMACIÓN TÉCNICA ============
    ESP_LOGI(TAG, "Configuración técnica:");
    ESP_LOGI(TAG, "- ADC: Canal %d, 12 bits, 3.3V máx", ADC_CHANNEL);
    ESP_LOGI(TAG, "- MUX Pines: A=%d, B=%d", MUX_PIN_A, MUX_PIN_B);
    ESP_LOGI(TAG, "- LED Status: Pin %d", LED_PIN);
    ESP_LOGI(TAG, "- Timer ADC: %d μs (%d Hz)", 1000000/SAMPLE_RATE_HZ, SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "- Protocolo datos: Start=0x%02X, Paquete=%d bytes", DATA_START_BYTE, DATA_PACKET_SIZE);
    
    ESP_LOGI(TAG, "Sistema listo para recibir comandos...");
}