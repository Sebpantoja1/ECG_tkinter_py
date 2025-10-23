
/*
  ECG_Control_Arduino.ino

  Este script se ejecuta en un Arduino Uno R3 y actúa como el controlador
  principal para el hardware de adquisición de ECG.

  Funcionalidades:
  1. Controla un multiplexor analógico (como el 74HC4052 o CD4052) para seleccionar
     entre 4 derivadas de ECG.
  2. Recibe comandos a través del puerto serial para cambiar el estado del MUX de
     forma manual o para iniciar/detener un modo de ciclado automático.
  3. Utiliza el Timer1 por hardware para el modo de ciclado automático, garantizando
     precisión y un funcionamiento no bloqueante.
  4. Recibe una señal de disparo (trigger) desde la aplicación de PC cuando se
     detecta un pico R y genera un pulso digital en un pin de salida.

  ---------------------------------------------------------------------
  PINOUT:
  - Pin 2: Salida digital para el selector S0 del multiplexor.
  - Pin 3: Salida digital para el selector S1 del multiplexor.
  - Pin 4: Salida digital para generar el pulso de disparo (trigger out).

  ---------------------------------------------------------------------
  PROTOCOLO SERIAL (Baud Rate: 9600):
  - Comandos recibidos desde el PC:
    - '0': Selecciona Derivada 0 (S1=0, S0=0).
    - '1': Selecciona Derivada 1 (S1=0, S0=1).
    - '2': Selecciona Derivada 2 (S1=1, S0=0).
    - '3': Selecciona Derivada 3 (S1=1, S0=1).
    - 'S': Inicia ('Start') el modo de ciclado automático.
    - 'X': Detiene ('Stop') el modo de ciclado automático.
    - 0x01 (Byte): Señal de disparo de pico R. Genera un pulso en TRIGGER_OUT_PIN.
*/

// --- Pines de Control ---
const int MUX_S0_PIN = 2;
const int MUX_S1_PIN = 3;
const int TRIGGER_OUT_PIN = 4;

// --- Configuración del Modo Automático ---
const unsigned long AUTO_CYCLE_INTERVAL_MS = 2000; // Cambia de derivada cada 2 segundos

// --- Variables Globales ---
// 'volatile' es crucial porque estas variables son modificadas por una
// Interrupt Service Routine (ISR) y leídas en otras partes del código.
volatile int currentMuxState = 0;
volatile bool autoModeEnabled = false;

void setup() {
  // Iniciar comunicación serial a la misma velocidad que la app de Python
  Serial.begin(9600);

  // Configurar pines de control como salidas
  pinMode(MUX_S0_PIN, OUTPUT);
  pinMode(MUX_S1_PIN, OUTPUT);
  pinMode(TRIGGER_OUT_PIN, OUTPUT);

  // Establecer el estado inicial del MUX y la salida de trigger
  setMuxState(currentMuxState);
  digitalWrite(TRIGGER_OUT_PIN, LOW);

  // --- Configuración del Timer1 por Hardware ---
  cli(); // Deshabilitar interrupciones temporalmente

  // Modo CTC (Clear Timer on Compare Match)
  TCCR1A = 0; // TCCR1A registrador a cero
  TCCR1B = 0; // TCCR1B registrador a cero
  TCNT1  = 0; // Inicializar contador a cero

  // Establecer el valor de comparación (OCR1A) para el intervalo deseado
  // Frecuencia del reloj = 16MHz. Queremos un intervalo de AUTO_CYCLE_INTERVAL_MS.
  // Usaremos un prescaler de 1024.
  // Frecuencia del Timer = 16,000,000 / 1024 = 15625 Hz.
  // Ticks necesarios = Frecuencia del Timer * Intervalo en segundos
  // Ticks = 15625 * (AUTO_CYCLE_INTERVAL_MS / 1000.0)
  unsigned long ocr_val = 15625 * (AUTO_CYCLE_INTERVAL_MS / 1000.0);
  OCR1A = ocr_val;

  // Activar modo CTC (WGM12) y prescaler de 1024 (CS12 y CS10)
  TCCR1B |= (1 << WGM12) | (1 << CS12) | (1 << CS10);

  // Habilitar la interrupción por comparación del Timer1
  TIMSK1 |= (1 << OCIE1A);

  sei(); // Rehabilitar interrupciones
  
  // El timer está configurado, pero no se inicia hasta recibir el comando 'S'
  // (En realidad, el timer ya está corriendo, pero la variable autoModeEnabled
  // controla si la ISR hace algo o no).
}

void loop() {
  // El loop principal solo se encarga de procesar los comandos seriales.
  // Todo lo demás es manejado por interrupciones, lo que hace el sistema
  // muy eficiente y receptivo.
  if (Serial.available() > 0) {
    handleSerialCommand();
  }
}

// --- Rutina de Servicio de Interrupción (ISR) para el Timer1 ---
ISR(TIMER1_COMPA_vect) {
  if (autoModeEnabled) {
    // Incrementar el estado del MUX y volver a 0 si supera 3
    currentMuxState++;
    if (currentMuxState > 3) {
      currentMuxState = 0;
    }
    // Actualizar los pines del MUX
    setMuxState(currentMuxState);
  }
}

// --- Función para manejar los comandos recibidos ---
void handleSerialCommand() {
  char command = Serial.read();

  switch (command) {
    case '0':
    case '1':
    case '2':
    case '3':
      autoModeEnabled = false; // Salir del modo automático
      currentMuxState = command - '0'; // Convertir char a int
      setMuxState(currentMuxState);
      Serial.print("MUX set to state: ");
      Serial.println(currentMuxState);
      break;

    case 'S':
      autoModeEnabled = true;
      Serial.println("Auto-cycle mode STARTED.");
      break;

    case 'X':
      autoModeEnabled = false;
      Serial.println("Auto-cycle mode STOPPED.");
      break;

    case 0x01: // Byte de disparo del pico R
      pulseTriggerPin();
      break;
  }
}

// --- Función para establecer el estado del multiplexor ---
void setMuxState(int state) {
  // Convierte el estado (0-3) a señales binarias para S0 y S1
  digitalWrite(MUX_S0_PIN, (state & 1));      // Bit 0 del estado
  digitalWrite(MUX_S1_PIN, ((state >> 1) & 1)); // Bit 1 del estado
}

// --- Función para generar un pulso de disparo ---
void pulseTriggerPin() {
  // Genera un pulso corto (100 microsegundos) en el pin de disparo.
  // Se usa delayMicroseconds para una pausa muy corta y precisa.
  digitalWrite(TRIGGER_OUT_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(TRIGGER_OUT_PIN, LOW);
}
