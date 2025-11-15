# Monitor ECG con Interfaz Gráfica y Control por Arduino

Este proyecto es una aplicación de escritorio en Python para visualizar y analizar señales de ECG. El sistema utiliza una arquitectura de dos microcontroladores: un **ESP32** actúa como un esclavo ADC para adquirir la señal, y un **Arduino Uno** gestiona el control del hardware (multiplexor y señales de disparo) basado en comandos enviados desde la aplicación.

## Arquitectura del Sistema

El sistema se compone de tres partes principales que se comunican vía puertos seriales:

1.  **Aplicación de PC (Python/Tkinter):**
    - Visualiza los datos del ECG en tiempo real.
    - Realiza el filtrado de la señal y la detección de picos R.
    - Proporciona una interfaz de usuario para enviar comandos de control.

2.  **ESP32 (Conectado a `SERIAL_PORT`, ej: `COM8`):**
    - Su única función es leer el conversor analógico-digital (ADC) a alta velocidad.
    - Envía los datos crudos del ADC a la aplicación de PC en paquetes binarios.

3.  **Arduino Uno (Conectado a `TRIGGER_SERIAL_PORT`, ej: `COM9`):**
    - Actúa como el controlador maestro del hardware.
    - Recibe comandos de un solo carácter desde la PC para cambiar la derivada del ECG (controlando un multiplexor).
    - Recibe una señal de disparo cuando la PC detecta un pico R y genera un pulso digital para sincronización externa.

## Características Principales

- **Visualización Dual:** Muestra simultáneamente la señal de ECG en crudo y la señal filtrada.
- **Procesamiento de Señal:** Filtros Notch y Pasa-banda para limpiar la señal.
- **Detección de Picos R y Cálculo de BPM:** Identifica picos R, los marca y calcula las pulsaciones por minuto.
- **Control Interactivo:** Permite ajustar en tiempo real la ganancia, offset, y parámetros de visualización y detección.
- **Control de Derivadas Remoto:** Envía comandos al Arduino para controlar un multiplexor (Modo Manual y Automático).
- **Salida de Disparo (Trigger):** Envía una señal al Arduino al detectar un pico R, que a su vez genera un pulso digital para sincronización.
- **Panel de Estado:** Informa sobre el estado de la conexión con los dispositivos, la calidad de la señal y la derivada actual.

## Estructura del Código

- `main.py`: Punto de entrada que inicia la aplicación.
- `uv.lock` / `pyproject.toml`: Archivos de gestión de dependencias para `uv`.
- `src/`: Directorio con el código fuente de la aplicación Python.
    - `app.py`: Lógica de la interfaz de usuario con Tkinter.
    - `serial_handler.py`: Gestiona la comunicación serial. Lee datos del ESP32 y envía comandos al Arduino.
    - `config.py`, `data_model.py`, `filters.py`, `peak_detection.py`: Módulos con responsabilidades separadas para configuración, estado, filtros y detección de picos.
- `microScripts/ECG_Control_Arduino/`: Contiene el código fuente (`.ino`) para el microcontrolador Arduino Uno.

## Código del Arduino

Dentro de la carpeta `microScripts/ECG_Control_Arduino/` se encuentra el archivo `ECG_Control_Arduino.ino`. Este código debe ser cargado en la placa Arduino Uno R3 utilizando el [Arduino IDE](https://www.arduino.cc/en/software).

El script está diseñado para ser eficiente, utilizando interrupciones y temporizadores de hardware para controlar el multiplexor y las señales de disparo sin bloquear el microcontrolador. El código está completamente comentado para facilitar su comprensión y modificación.

## Instalación y Ejecución (Aplicación de PC)

Para gestionar las dependencias se utiliza `uv`, un gestor de paquetes rápido para Python.

1.  **Instalar `uv`:**
    ```bash
    pip install uv
    ```

2.  **Crear y Activar Entorno Virtual:**
    ```bash
    uv venv
    # En Windows:
    .\.venv\Scripts\activate
    # En macOS/Linux:
    # source .venv/bin/activate
    ```

3.  **Instalar Dependencias:**
    ```bash
    uv pip sync
    ```

4.  **Ejecutar la Aplicación:**
    ```bash
    uv run main.py
    ```

## Configuración

Antes de ejecutar, asegúrate de que los puertos en `src/config.py` coincidan con tu configuración:

- `SERIAL_PORT`: Puerto COM del **ESP32** (ej. "COM8").
- `TRIGGER_SERIAL_PORT`: Puerto COM del **Arduino** (ej. "COM9").
- `BAUD_RATE`: Velocidad para el ESP32.
- `TRIGGER_BAUD_RATE`: Velocidad para el Arduino (debe ser **9600** para coincidir con el script `.ino`).
- `SAMPLE_RATE`: Frecuencia de muestreo de tu ESP32.

## Firmware ESP32 (ADC Slave con PlatformIO)

El firmware para el ESP32, ubicado en `microScripts/ADC-slaveESP32-uart/`, está configurado como un proyecto de **PlatformIO** que utiliza el framework **ESP-IDF**. Su única función es actuar como un esclavo ADC de alta velocidad, enviando los datos crudos a la aplicación de PC.

### Configuración del Entorno de Desarrollo

Para trabajar con el firmware, se recomienda usar **Visual Studio Code** con la extensión oficial de **PlatformIO IDE**.

1.  **Instalar la extensión de PlatformIO:**
    -   Abre Visual Studio Code.
    -   Ve al panel de Extensiones (`Ctrl+Shift+X`).
    -   Busca "PlatformIO IDE" e instálala.

2.  **Abrir el Proyecto:**
    -   Abre la carpeta raíz `ECG_tkinter_py` en VS Code.
    -   PlatformIO detectará automáticamente el proyecto del ESP32. Puedes acceder a sus funcionalidades desde la barra de herramientas de PlatformIO (icono de alien en la barra lateral).

### Comandos de Compilación

Puedes compilar, cargar y limpiar el proyecto usando la interfaz de PlatformIO en VS Code o a través de la línea de comandos.

1.  **Abrir una terminal de PlatformIO:**
    -   Desde la barra de herramientas de PlatformIO, selecciona `Miscellaneous > New Terminal`.
    -   Esto abrirá una terminal con el entorno de PlatformIO activado.

2.  **Limpiar la build (Clean):**
    -   Para eliminar todos los archivos de compilaciones anteriores y asegurar una build limpia, ejecuta:
    ```bash
    pio run --target clean
    ```

3.  **Compilar el proyecto (Build):**
    -   Para compilar el firmware sin cargarlo al dispositivo:
    ```bash
    pio run
    ```

4.  **Compilar y Cargar (Upload):**
    -   Para compilar y cargar el firmware al ESP32 conectado:
    ```bash
    pio run --target upload
    ```
