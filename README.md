# Monitor ECG con Interfaz Gráfica en Tkinter

Este proyecto es una aplicación de escritorio desarrollada en Python con Tkinter para la visualización y análisis en tiempo real de señales de electrocardiograma (ECG) adquiridas a través de un puerto serial, comúnmente desde un microcontrolador como el ESP32.

## Características Principales

- **Visualización Dual:** Muestra simultáneamente la señal de ECG en crudo y la señal filtrada.
- **Procesamiento de Señal:**
    - Filtro Notch para eliminar la interferencia de la línea eléctrica (60Hz).
    - Filtro Pasa-banda para aislar las frecuencias de interés del ECG.
- **Detección de Picos R:** Identifica los picos R en la señal filtrada y los marca en la gráfica.
- **Cálculo de BPM:** Estima y muestra las pulsaciones por minuto (BPM) en tiempo real.
- **Control Interactivo:**
    - Ajuste de ganancia y offset de la señal.
    - Modificación de los parámetros de la gráfica (amplitud, ventana de tiempo).
    - Ajuste del umbral y la distancia para la detección de picos R.
- **Control de Derivadas:** Envía comandos a través del puerto serial para controlar un multiplexor y cambiar entre diferentes derivadas de ECG (Modo Manual y Automático).
- **Salida de Disparo (Trigger):** Envía una señal a través de un puerto serial secundario al detectar un pico R, útil para la sincronización con otros equipos.
- **Panel de Estado:** Informa sobre el estado de la conexión, la calidad de la señal, la derivada actual y el estado de los electrodos.

## Estructura del Código

El código ha sido refactorizado para mejorar su organización, escalabilidad y mantenibilidad. La estructura principal es la siguiente:

- `main.py`: Es el punto de entrada de la aplicación. Su única responsabilidad es iniciar la interfaz gráfica.
- `uv.lock` / `pyproject.toml`: Archivos de gestión de dependencias utilizados por `uv`.
- `src/`: Directorio que contiene todo el código fuente de la aplicación.
    - `__init__.py`: Convierte `src` en un paquete de Python.
    - `app.py`: Contiene la clase `ECGApp`, que gestiona toda la interfaz de usuario con Tkinter y la orquestación de los demás componentes.
    - `config.py`: Almacena todas las constantes y parámetros de configuración (puertos serial, baud rates, etc.).
    - `data_model.py`: Define la clase `AppState`, que centraliza y gestiona el estado compartido de la aplicación (buffers de datos, estado de conexión, etc.).
    - `filters.py`: Implementa la clase `ECGFilters` para el procesamiento y filtrado de la señal.
    - `peak_detection.py`: Contiene las funciones para la detección de picos R y el cálculo de BPM.
    - `serial_handler.py`: Gestiona toda la comunicación a través de los puertos seriales (lectura de datos, envío de comandos y señales de disparo).

## Instalación y Ejecución con `uv`

Para gestionar las dependencias y el entorno de desarrollo, este proyecto utiliza `uv`, una herramienta moderna y extremadamente rápida para Python.

### ¿Qué es `uv`?

`uv` es un instalador y gestor de entornos virtuales de Python desarrollado por Astral (los creadores de `ruff`). Es un reemplazo de alto rendimiento para herramientas como `pip` y `venv`. El archivo `uv.lock` es un "lockfile" que asegura que siempre se instalen las mismas versiones de las dependencias, garantizando la reproducibilidad del entorno.

### Pasos para la Ejecución

1.  **Instalar `uv`:**
    Si aún no tienes `uv`, puedes instalarlo usando `pip`:
    ```bash
    pip install uv
    ```

2.  **Crear y Activar el Entorno Virtual:**
    Navega a la raíz del proyecto y usa `uv` para crear un entorno virtual. `uv` lo creará en un directorio `.venv` por defecto.
    ```bash
    uv venv
    ```
    Luego, activa el entorno. En Windows:
    ```bash
    .\.venv\Scripts\activate
    ```
    En macOS/Linux:
    ```bash
    source .venv/bin/activate
    ```

3.  **Instalar Dependencias:**
    Con el entorno activado, usa `uv` para instalar todas las dependencias especificadas en `pyproject.toml` y `uv.lock`. El comando `sync` asegura que tu entorno sea una réplica exacta del entorno de desarrollo.
    ```bash
    uv pip sync
    ```
    Si `uv.lock` no existiera, podrías usar `uv pip install .` para instalar las dependencias desde `pyproject.toml`.

4.  **Ejecutar la Aplicación:**
    Una vez instaladas las dependencias, puedes ejecutar la aplicación:
    ```bash
    python main.py
    ```

## Configuración

Antes de ejecutar la aplicación, es posible que necesites ajustar la configuración en `src/config.py`:

- `SERIAL_PORT`: Define el puerto COM para la adquisición de datos del ECG (ej. "COM8").
- `TRIGGER_SERIAL_PORT`: Define el puerto COM para la señal de disparo (ej. "COM9").
- `BAUD_RATE`: Velocidad de transmisión para el puerto de datos.
- `SAMPLE_RATE`: Frecuencia de muestreo de tu dispositivo.

Asegúrate de que estos valores coincidan con la configuración de tu hardware.
