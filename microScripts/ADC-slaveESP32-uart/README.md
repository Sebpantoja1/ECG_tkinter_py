# Guía de Configuración - Sistema ECG ESP32

## 📋 **Índice**
- [Archivo `config.h`](#archivo-configh) - Configuraciones principales
- [Archivo `functions.h`](#archivo-functionsh) - Declaraciones de funciones  
- [Archivo `functions.c`](#archivo-functionsc) - Implementación del sistema
- [Parámetros Configurables](#parámetros-configurables) - Lo que puedes cambiar
- [Parámetros Críticos](#parámetros-críticos) - ¡No tocar!
- [Ejemplos de Configuración](#ejemplos-de-configuración)

---

## 📁 **Archivo `config.h`**

### **Propósito:**
Contiene **todas** las configuraciones, constantes, estructuras y declaraciones globales del sistema. Es el "centro de control" del proyecto.

### **Secciones organizadas:**

#### 🔧 **Configuraciones del Sistema**
```c
#define SAMPLE_RATE_HZ 2000        // ✅ CONFIGURABLE: Frecuencia de muestreo ADC
#define UART_BAUD_RATE 115200      // ✅ CONFIGURABLE: Velocidad UART
#define ADC_QUEUE_SIZE 200         // ✅ CONFIGURABLE: Tamaño cola ADC
```

#### 📡 **Pines GPIO**
```c
#define ADC_CHANNEL ADC_CHANNEL_0  // ✅ CONFIGURABLE: Canal ADC a usar
#define MUX_PIN_A GPIO_NUM_25      // ✅ CONFIGURABLE: Pin control multiplexor A
#define MUX_PIN_B GPIO_NUM_26      // ✅ CONFIGURABLE: Pin control multiplexor B
#define LED_PIN GPIO_NUM_2         // ✅ CONFIGURABLE: Pin LED de estado
```

#### ⚖️ **Prioridades de Tareas**
```c
#define ADC_TASK_PRIORITY 6        // ✅ CONFIGURABLE: Prioridad tarea ADC
#define UART_TASK_PRIORITY 5       // ✅ CONFIGURABLE: Prioridad tarea UART
#define MUX_TASK_PRIORITY 4        // ✅ CONFIGURABLE: Prioridad tarea MUX
```

#### ⏱️ **Timeouts y Delays**
```c
#define MUX_DEFAULT_DELAY_MS 10000 // ✅ CONFIGURABLE: Delay inicial multiplexor
#define MUTEX_TIMEOUT_MS 100       // ✅ CONFIGURABLE: Timeout de mutex
#define QUEUE_TIMEOUT_MS 1         // ⚠️ CRÍTICO: Timeout de colas
```

---

## 📄 **Archivo `functions.h`**

### **Propósito:**
Contiene únicamente las **declaraciones** de todas las funciones del sistema. Es como un "índice" de funciones disponibles.

### **Estructura:**
```c
#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "config.h"  // Importa todas las configuraciones

// Declaraciones organizadas por categoría:
// - Inicialización
// - Tareas principales  
// - Callbacks
// - Control de hardware
// - Utilidades
```

### **No contiene configuraciones** - Solo referencias a funciones.

---

## 📝 **Archivo `functions.c`**

### **Propósito:**
Contiene la **implementación completa** de todas las funciones del sistema. Es donde ocurre toda la lógica del programa.

### **Secciones principales:**

#### 🚀 **Inicialización del Sistema**
- `system_init()` - Maestro que inicializa todo
- `adc_init()` - Configuración del ADC
- `uart_init()` - Configuración del UART
- `gpio_init()` - Configuración de pines
- `timers_init()` - Configuración de timers

#### 🔄 **Tareas Principales**
- `adc_acquisition_task()` - Adquisición a 2000 Hz
- `uart_communication_task()` - Comunicación serie
- `mux_control_task()` - Control del multiplexor

#### 🎛️ **Control de Hardware**
- `set_mux_state()` - Cambiar estado del multiplexor
- `process_mux_command()` - Procesar comandos UART
- `send_data_packet()` - Enviar datos por UART

---

## ⚙️ **Parámetros Configurables**

### 🟢 **SEGUROS de cambiar (sin romper código):**

#### **Velocidades y Frecuencias:**
```c
#define SAMPLE_RATE_HZ 2000        // Rango: 100-5000 Hz
#define UART_BAUD_RATE 115200      // Valores: 9600, 57600, 115200, 230400
```

#### **Pines GPIO (cualquier pin libre del ESP32):**
```c
#define MUX_PIN_A GPIO_NUM_25      // Cualquier pin digital libre
#define MUX_PIN_B GPIO_NUM_26      // Cualquier pin digital libre  
#define LED_PIN GPIO_NUM_2         // Cualquier pin digital libre
```

#### **Canal ADC:**
```c
#define ADC_CHANNEL ADC_CHANNEL_0  // Opciones: ADC_CHANNEL_0 a ADC_CHANNEL_7
```

#### **Delays del Multiplexor:**
```c
#define MUX_DEFAULT_DELAY_MS 10000 // Rango: 100-60000 ms
#define MUX_MIN_DELAY_MS 100       // Mínimo delay permitido
#define MUX_MAX_DELAY_MS 60000     // Máximo delay permitido
```

#### **Tamaños de Buffer:**
```c
#define BUF_SIZE 1024              // Rango: 512-2048
#define RD_BUF_SIZE 256            // Rango: 128-512
#define ADC_QUEUE_SIZE 200         // Rango: 50-500
```

#### **Prioridades de Tareas:**
```c
#define ADC_TASK_PRIORITY 6        // Rango: 1-10 (6 = alta prioridad)
#define UART_TASK_PRIORITY 5       // Rango: 1-10 (5 = media-alta)
#define MUX_TASK_PRIORITY 4        // Rango: 1-10 (4 = media)
```

#### **Tamaños de Stack:**
```c
#define ADC_STACK_SIZE 4096        // Rango: 2048-8192
#define UART_STACK_SIZE 4096       // Rango: 2048-8192  
#define MUX_STACK_SIZE 2048        // Rango: 1024-4096
```

### 🟡 **CUIDADO al cambiar (pueden afectar rendimiento):**

#### **Timeouts de Sistema:**
```c
#define MUTEX_TIMEOUT_MS 100       // No bajar de 50ms
#define UART_CMD_TIMEOUT_MS 10     // No bajar de 5ms
```

#### **Configuraciones ADC:**
```c
#define ADC_BITWIDTH_USED ADC_BITWIDTH_12  // Mantener en 12 bits
#define ADC_ATTEN_USED ADC_ATTEN_DB_11     // Para 0-3.3V mantener
```

---

## 🔴 **Parámetros Críticos (NO CAMBIAR)**

### **Protocolo de Datos:**
```c
#define DATA_START_BYTE 0xAA       // ❌ CRÍTICO: Protocolo UART
#define DATA_PACKET_SIZE 4         // ❌ CRÍTICO: Tamaño de paquete  
#define QUEUE_TIMEOUT_MS 1         // ❌ CRÍTICO: Rendimiento tiempo real
```

### **Configuraciones UART:**
```c
#define UART_PORT UART_NUM_0       // ❌ CRÍTICO: Puerto fijo
#define UART_DATA_BITS_USED UART_DATA_8_BITS   // ❌ CRÍTICO: Protocolo
#define UART_PARITY_USED UART_PARITY_DISABLE   // ❌ CRÍTICO: Sin paridad
```

### **Estructuras de Datos:**
```c
typedef enum { ... } mux_state_t;  // ❌ CRÍTICO: Estados del multiplexor
typedef struct { ... } adc_data_t; // ❌ CRÍTICO: Estructura de datos ADC
```

---

## 💡 **Ejemplos de Configuración**

### **Ejemplo 1: Cambiar frecuencia de muestreo a 1000 Hz**
```c
// En config.h - línea ~16
#define SAMPLE_RATE_HZ 1000        // Era 2000, ahora 1000 Hz
```

### **Ejemplo 2: Usar pines diferentes para el multiplexor**
```c  
// En config.h - líneas ~24-25
#define MUX_PIN_A GPIO_NUM_18      // Era GPIO_NUM_25
#define MUX_PIN_B GPIO_NUM_19      // Era GPIO_NUM_26
```

### **Ejemplo 3: Aumentar velocidad UART**
```c
// En config.h - línea ~17
#define UART_BAUD_RATE 230400      // Era 115200
```

### **Ejemplo 4: Usar canal ADC diferente**
```c
// En config.h - línea ~23
#define ADC_CHANNEL ADC_CHANNEL_3  // Era ADC_CHANNEL_0
```

### **Ejemplo 5: Cambiar delay inicial del multiplexor**
```c
// En config.h - línea ~52
#define MUX_DEFAULT_DELAY_MS 5000  // Era 10000 ms, ahora 5 segundos
```

### **Ejemplo 6: Ajustar prioridades para mejor rendimiento**
```c
// En config.h - líneas ~39-41
#define ADC_TASK_PRIORITY 7        // Mayor prioridad para ADC
#define UART_TASK_PRIORITY 6       // Aumentar prioridad UART
#define MUX_TASK_PRIORITY 3        // Bajar prioridad MUX
```

---

## 📋 **Checklist antes de Cambiar Configuraciones**

### ✅ **Antes de modificar config.h:**

1. **Hacer respaldo** del archivo original
2. **Cambiar solo UNA configuración** a la vez
3. **Compilar y probar** después de cada cambio
4. **Verificar logs** en el monitor serie
5. **Probar comandos UART** después de cambios

### ⚠️ **Señales de problemas:**

- **ESP32 se reinicia constantemente** → Stack insuficiente o prioridades incorrectas
- **Comandos no responden** → Timeouts muy bajos o UART mal configurado
- **Datos corruptos** → Frecuencia muy alta o buffers insuficientes
- **LED no parpadea** → Pin incorrecto o multiplexor mal configurado

### 🔧 **Configuraciones recomendadas por uso:**

#### **Para ECG de alta precisión:**
```c
#define SAMPLE_RATE_HZ 2000
#define ADC_BITWIDTH_USED ADC_BITWIDTH_12
#define ADC_TASK_PRIORITY 6
```

#### **Para pruebas/desarrollo:**
```c
#define SAMPLE_RATE_HZ 500
#define MUX_DEFAULT_DELAY_MS 3000
#define UART_BAUD_RATE 115200
```

#### **Para análisis en tiempo real:**
```c
#define ADC_QUEUE_SIZE 100
#define QUEUE_TIMEOUT_MS 1
#define ADC_TASK_PRIORITY 7
```

---

## 🎯 **Resumen de Archivos**

| Archivo | Qué contiene | Se puede modificar |
|---------|-------------|-------------------|
| **`config.h`** | Configuraciones, defines, estructuras | ✅ Sí - Parámetros seguros |
| **`functions.h`** | Solo declaraciones de funciones | ❌ No - Solo referencias |
| **`functions.c`** | Implementación completa | ⚠️ Solo expertos |

### 🔑 **Regla de Oro:**
> **"Solo modifica `config.h` y únicamente los parámetros marcados como CONFIGURABLES"**

¡Con esta guía puedes personalizar el sistema sin romper el código! 🚀