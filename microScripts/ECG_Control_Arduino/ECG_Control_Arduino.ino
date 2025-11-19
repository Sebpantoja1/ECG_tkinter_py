#include <TimerOne.h>
#include <string.h>

// ===================== Protocolo Serial =====================
#define CMD_TRIGGER 0x01
#define CMD_MUX_0   '0'
#define CMD_MUX_1   '1'
#define CMD_MUX_2   '2'
#define CMD_MUX_3   '3'
#define CMD_MUX_START 'S'
#define CMD_MUX_STOP  'X'
#define CMD_CONFIG 'C'  // Configuración: CFG:vcap:pos:neg

// ===================== Pines Relés =====================
#define PIN_RELAY_CHARGE      2
#define PIN_RELAY_DISCHARGE   3
#define PIN_RELAY_POL_POS     4
#define PIN_RELAY_POL_NEG     5

// ===================== Pines ADC =====================
#define PIN_ADC_CHARGE        A0
#define PIN_ADC_DISCHARGE_POS A1
#define PIN_ADC_DISCHARGE_NEG A2
#define PIN_ADC_CURRENT_SHUNT A3  // Nuevo pin para medir corriente

// ===================== Pines MUX =====================
#define PIN_MUX_S0            6
#define PIN_MUX_S1            7

// ===================== Parámetros Cardioversor =====================
static const float R_SHUNT = 0.1f;         // ohm
static const float K_DIV_CHARGE = 0.122f;  // divisor V_cap (Vadc = Vcap * K_DIV)
static const float V_AREF = 5.0f;
static const uint16_t ADC_MAX = 1023;
static const float ADC_ATTENUATION = 5.0f; // factor atenuación entrada ADC ×5
static const float C_CAP = 470e-6f;        // 470 µF capacitor

// Objetivos de PORCENTAJE de energía por fase (0-100%)
static float g_target_percent_pos = 50.0f;  // % energía fase +
static float g_target_percent_neg = 35.0f;  // % energía fase -
// Reserva 15% mínimo para protección capacitor
static const float MIN_RESERVE_PERCENT = 15.0f;

// Voltaje objetivo de carga
static float g_target_vcap_V = 20.0f;

// Energías calculadas dinámicamente
static volatile float g_E_total_J = 0.0f;       // energía total almacenada al armar
static volatile float g_target_energy_pos_J = 0.0f;
static volatile float g_target_energy_neg_J = 0.0f;

// Temporizaciones (ms)
static const uint32_t DISCH_POS_MS = 40;
static const uint32_t DISCH_NEG_MS = 40;
static const uint32_t SAFE_MS = 1;
static const uint32_t CHARGE_TIMEOUT_MS = 4000;
static const uint32_t REFRACT_MS = 200;

// ===================== FSM Estados =====================
enum DefibState {
  DEFIB_IDLE,
  DEFIB_CHARGING,
  DEFIB_ARMED,
  DEFIB_DISCHARGING_POS,
  DEFIB_DISCHARGING_NEG,
  DEFIB_SAFE,
  DEFIB_REFRACT
};

static volatile DefibState g_state = DEFIB_IDLE;
static volatile bool g_triggerBusy = false;
static volatile uint32_t g_phaseStartMs = 0;
static volatile uint32_t g_chargeStartMs = 0;
static volatile uint32_t g_refractStartMs = 0;

// Energía integrada
static volatile float g_energy_pos_J = 0.0f;
static volatile float g_energy_neg_J = 0.0f;

// ===================== ADC Timer =====================
static const unsigned long ADC_SAMPLE_PERIOD_US = 1000; // 1 kHz
static volatile bool g_adc_timer_active = false;
static volatile uint16_t g_last_vcap_counts = 0;
static uint32_t g_last_adc_us = 0;
static float g_P_prev = 0.0f;

// ===================== Integración Simpson =====================
static const uint8_t SIMPSON_BUFFER_SIZE = 10;
static float g_power_buffer[SIMPSON_BUFFER_SIZE];
static uint8_t g_buffer_idx = 0;
static bool g_buffer_full = false;

// Integración de Simpson: más preciso que trapezoidal
// I = (dt/3) * [P0 + 4*P1 + 2*P2 + 4*P3 + ... + Pn]
float simpson_integrate(float dt) {
  if (!g_buffer_full) return 0.0f;
  
  float integral = g_power_buffer[0];
  
  for (uint8_t i = 1; i < SIMPSON_BUFFER_SIZE - 1; i++) {
    float coeff = (i % 2 == 1) ? 4.0f : 2.0f;
    integral += coeff * g_power_buffer[i];
  }
  
  integral += g_power_buffer[SIMPSON_BUFFER_SIZE - 1];
  integral *= (dt / 3.0f);
  
  return integral;
}

// ===================== MUX Estado =====================
static volatile uint8_t g_mux_state = 0;
static volatile bool g_mux_auto = false;
static volatile uint32_t g_mux_auto_ms = 3000;
static volatile uint32_t g_mux_last_switch = 0;

// ===================== Buffer Serial =====================
static char g_serial_buffer[64];
static uint8_t g_buffer_len = 0;

// ===================== Status Report =====================
static uint32_t g_last_status_report = 0;
static const uint32_t STATUS_REPORT_INTERVAL_MS = 1000;

// ===================== Utilidades ADC =====================
static inline float adc_to_volts(uint16_t counts) {
  // Incluye factor atenuación ×5
  return (counts / (float)ADC_MAX) * V_AREF * ADC_ATTENUATION;
}

static inline float adc_to_vcap(uint16_t counts) {
  float vadc = (counts / (float)ADC_MAX) * V_AREF; // sin atenuación, directo del divisor
  return vadc / K_DIV_CHARGE;
}

// Cálculo de energía total del capacitor: E = 0.5 * C * V²
static inline float capacitor_energy(float vcap) {
  return 0.5f * C_CAP * vcap * vcap;
}

// ===================== ISR Timer ADC =====================
void adc_timer_isr() {
  if (!g_adc_timer_active) return;

  uint32_t now_us = micros();
  float dt = (g_last_adc_us == 0) ? 0.001f : (now_us - g_last_adc_us) * 1e-6f;
  g_last_adc_us = now_us;

  if (g_state == DEFIB_DISCHARGING_POS) {
    uint16_t vload_counts = analogRead(PIN_ADC_DISCHARGE_POS);
    float Vload = adc_to_volts(vload_counts);
    
    // Leer corriente desde shunt dedicado
    uint16_t ishunt_counts = analogRead(PIN_ADC_CURRENT_SHUNT);
    float V_shunt = adc_to_volts(ishunt_counts);
    float I = V_shunt / R_SHUNT;
    
    float P = Vload * I;
    
    // Agregar a buffer Simpson
    g_power_buffer[g_buffer_idx] = P;
    g_buffer_idx++;
    if (g_buffer_idx >= SIMPSON_BUFFER_SIZE) {
      g_buffer_idx = 0;
      g_buffer_full = true;
    }
    
    // Integrar cada N muestras
    if (g_buffer_full) {
      g_energy_pos_J += simpson_integrate(dt * SIMPSON_BUFFER_SIZE);
      // Reset buffer
      g_buffer_full = false;
      g_buffer_idx = 0;
    }
  } 
  else if (g_state == DEFIB_DISCHARGING_NEG) {
    uint16_t vload_counts = analogRead(PIN_ADC_DISCHARGE_NEG);
    float Vload = adc_to_volts(vload_counts);
    
    // Leer corriente desde shunt dedicado
    uint16_t ishunt_counts = analogRead(PIN_ADC_CURRENT_SHUNT);
    float V_shunt = adc_to_volts(ishunt_counts);
    float I = V_shunt / R_SHUNT;
    
    float P = Vload * I;
    
    // Agregar a buffer Simpson
    g_power_buffer[g_buffer_idx] = P;
    g_buffer_idx++;
    if (g_buffer_idx >= SIMPSON_BUFFER_SIZE) {
      g_buffer_idx = 0;
      g_buffer_full = true;
    }
    
    // Integrar cada N muestras
    if (g_buffer_full) {
      g_energy_neg_J += simpson_integrate(dt * SIMPSON_BUFFER_SIZE);
      // Reset buffer
      g_buffer_full = false;
      g_buffer_idx = 0;
    }
  }
}

// ===================== Control Relés =====================
static inline void set_all_relays_low() {
  digitalWrite(PIN_RELAY_CHARGE, LOW);
  digitalWrite(PIN_RELAY_DISCHARGE, LOW);
  digitalWrite(PIN_RELAY_POL_POS, LOW);
  digitalWrite(PIN_RELAY_POL_NEG, LOW);
}

// ===================== MUX Apply =====================
static inline void mux_apply(uint8_t state) {
  digitalWrite(PIN_MUX_S0, (state & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_MUX_S1, (state & 0x02) ? HIGH : LOW);
  Serial.print("MUX=");
  switch(state) {
    case 0: Serial.println("I"); break;
    case 1: Serial.println("II"); break;
    case 2: Serial.println("III"); break;
    case 3: Serial.println("aVR"); break;
    default: Serial.println("?"); break;
  }
}

// ===================== FSM Cardioversor =====================
void fsm_step() {
  uint32_t now = millis();

  switch (g_state) {
    case DEFIB_IDLE:
      break;

    case DEFIB_CHARGING: {
      digitalWrite(PIN_RELAY_CHARGE, HIGH);
      uint16_t c = analogRead(PIN_ADC_CHARGE);
      g_last_vcap_counts = c;
      float vcap = adc_to_vcap(c);
      
      if (vcap >= g_target_vcap_V) {
        digitalWrite(PIN_RELAY_CHARGE, LOW);
        
        // Calcular energía total y objetivos por fase
        g_E_total_J = capacitor_energy(vcap);
        
        // Validar porcentajes
        float total_percent = g_target_percent_pos + g_target_percent_neg;
        if (total_percent > (100.0f - MIN_RESERVE_PERCENT)) {
          Serial.println("[FSM] ERROR: Porcentajes exceden límite (deben sumar <85%)");
          set_all_relays_low();
          g_state = DEFIB_REFRACT;
          g_refractStartMs = now;
          g_triggerBusy = false;
          break;
        }
        
        g_target_energy_pos_J = g_E_total_J * (g_target_percent_pos / 100.0f);
        g_target_energy_neg_J = g_E_total_J * (g_target_percent_neg / 100.0f);
        
        Serial.print("[FSM] ARMED Vcap=");
        Serial.print(vcap, 2);
        Serial.print("V E_total=");
        Serial.print(g_E_total_J, 4);
        Serial.print("J E+_target=");
        Serial.print(g_target_energy_pos_J, 4);
        Serial.print("J (");
        Serial.print(g_target_percent_pos, 1);
        Serial.print("%) E-_target=");
        Serial.print(g_target_energy_neg_J, 4);
        Serial.print("J (");
        Serial.print(g_target_percent_neg, 1);
        Serial.println("%)");
        
        g_state = DEFIB_ARMED;
      } else if ((now - g_chargeStartMs) >= CHARGE_TIMEOUT_MS) {
        digitalWrite(PIN_RELAY_CHARGE, LOW);
        set_all_relays_low();
        g_state = DEFIB_REFRACT;
        g_refractStartMs = now;
        g_triggerBusy = false;
        Serial.println("[FSM] FAIL:CHARGE_TIMEOUT");
      }
    } break;

    case DEFIB_ARMED: {
      g_state = DEFIB_DISCHARGING_POS;
      g_phaseStartMs = now;
      g_energy_pos_J = 0.0f;
      g_P_prev = 0.0f;
      g_last_adc_us = 0;
      g_buffer_idx = 0;
      g_buffer_full = false;
      g_adc_timer_active = true;
      
      digitalWrite(PIN_RELAY_DISCHARGE, LOW);
      digitalWrite(PIN_RELAY_POL_POS, HIGH);
      digitalWrite(PIN_RELAY_POL_NEG, LOW);
      delay(5);
      digitalWrite(PIN_RELAY_DISCHARGE, HIGH);
      Serial.println("[FSM] DISCH_POS start");
    } break;

    case DEFIB_DISCHARGING_POS: {
      if (g_energy_pos_J >= g_target_energy_pos_J || (now - g_phaseStartMs) >= DISCH_POS_MS) {
        digitalWrite(PIN_RELAY_DISCHARGE, LOW);
        float percent_delivered = (g_E_total_J > 0) ? (g_energy_pos_J / g_E_total_J * 100.0f) : 0.0f;
        Serial.print("[FSM] DISCH_POS end E=");
        Serial.print(g_energy_pos_J, 4);
        Serial.print("J (");
        Serial.print(percent_delivered, 1);
        Serial.println("% del total)");
        
        g_state = DEFIB_DISCHARGING_NEG;
        g_phaseStartMs = now;
        g_energy_neg_J = 0.0f;
        g_P_prev = 0.0f;
        g_last_adc_us = 0;
        g_buffer_idx = 0;
        g_buffer_full = false;
        
        digitalWrite(PIN_RELAY_POL_POS, LOW);
        delay(5);
        digitalWrite(PIN_RELAY_POL_NEG, HIGH);
        delay(5);
        digitalWrite(PIN_RELAY_DISCHARGE, HIGH);
        Serial.println("[FSM] DISCH_NEG start");
      }
    } break;

    case DEFIB_DISCHARGING_NEG: {
      if (g_energy_neg_J >= g_target_energy_neg_J || (now - g_phaseStartMs) >= DISCH_NEG_MS) {
        digitalWrite(PIN_RELAY_DISCHARGE, LOW);
        digitalWrite(PIN_RELAY_POL_NEG, LOW);
        g_adc_timer_active = false;
        
        float percent_delivered = (g_E_total_J > 0) ? (g_energy_neg_J / g_E_total_J * 100.0f) : 0.0f;
        float total_delivered = g_energy_pos_J + g_energy_neg_J;
        float total_percent = (g_E_total_J > 0) ? (total_delivered / g_E_total_J * 100.0f) : 0.0f;
        
        Serial.print("[FSM] DISCH_NEG end E=");
        Serial.print(g_energy_neg_J, 4);
        Serial.print("J (");
        Serial.print(percent_delivered, 1);
        Serial.println("% del total)");
        Serial.print("[FSM] Total entregado: ");
        Serial.print(total_delivered, 4);
        Serial.print("J (");
        Serial.print(total_percent, 1);
        Serial.print("% de ");
        Serial.print(g_E_total_J, 4);
        Serial.println("J)");
        
        g_state = DEFIB_SAFE;
        g_phaseStartMs = now;
        Serial.println("[FSM] SAFE");
      }
    } break;

    case DEFIB_SAFE: {
      if ((now - g_phaseStartMs) >= SAFE_MS) {
        set_all_relays_low();
        g_state = DEFIB_REFRACT;
        g_refractStartMs = now;
        g_triggerBusy = false;
        Serial.println("[FSM] SUCCESS -> REFRACT");
      }
    } break;

    case DEFIB_REFRACT: {
      if ((now - g_refractStartMs) >= REFRACT_MS) {
        g_state = DEFIB_IDLE;
        Serial.println("[FSM] IDLE");
      }
    } break;
  }
}

// ===================== Parser Serial =====================
void parse_config_command(char* buf) {
  // Parsear "CFG:20.0:50.0:35.0"
  char* token = strtok(buf, ":");
  if (strcmp(token, "CFG") != 0) return;
  
  token = strtok(NULL, ":");
  if (token) g_target_vcap_V = atof(token);
  
  token = strtok(NULL, ":");
  if (token) g_target_percent_pos = atof(token);
  
  token = strtok(NULL, ":");
  if (token) g_target_percent_neg = atof(token);
  
  Serial.print("[CFG] Updated: Vcap=");
  Serial.print(g_target_vcap_V);
  Serial.print("V Pos=");
  Serial.print(g_target_percent_pos);
  Serial.print("% Neg=");
  Serial.print(g_target_percent_neg);
  Serial.println("%");
}

void handle_serial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Comandos de un byte (legacy)
    if (c == CMD_TRIGGER) {
      if (g_state == DEFIB_IDLE && !g_triggerBusy) {
        g_triggerBusy = true;
        g_state = DEFIB_CHARGING;
        g_chargeStartMs = millis();
        g_energy_pos_J = 0.0f;
        g_energy_neg_J = 0.0f;
        g_buffer_idx = 0;
        g_buffer_full = false;
        set_all_relays_low();
        Serial.println("[CMD] TRIGGER accepted -> CHARGING");
      } else {
        Serial.println("[CMD] TRIGGER rejected (BUSY/REFRACT)");
      }
      continue;
    }
    else if (c == CMD_MUX_0) {
      g_mux_state = 0;
      g_mux_auto = false;
      mux_apply(g_mux_state);
      continue;
    }
    else if (c == CMD_MUX_1) {
      g_mux_state = 1;
      g_mux_auto = false;
      mux_apply(g_mux_state);
      continue;
    }
    else if (c == CMD_MUX_2) {
      g_mux_state = 2;
      g_mux_auto = false;
      mux_apply(g_mux_state);
      continue;
    }
    else if (c == CMD_MUX_3) {
      g_mux_state = 3;
      g_mux_auto = false;
      mux_apply(g_mux_state);
      continue;
    }
    else if (c == CMD_MUX_START) {
      g_mux_auto = true;
      g_mux_last_switch = millis();
      Serial.println("[CMD] MUX AUTO ON");
      continue;
    }
    else if (c == CMD_MUX_STOP) {
      g_mux_auto = false;
      Serial.println("[CMD] MUX AUTO OFF");
      continue;
    }
    
    // Comandos multi-byte (nuevos)
    if (c == '\n') {
      g_serial_buffer[g_buffer_len] = '\0';
      
      // Parsear comando
      if (strncmp(g_serial_buffer, "CFG:", 4) == 0) {
        parse_config_command(g_serial_buffer);
      }
      
      g_buffer_len = 0;
    } else {
      if (g_buffer_len < sizeof(g_serial_buffer) - 1) {
        g_serial_buffer[g_buffer_len++] = c;
      }
    }
  }
}

// ===================== MUX Auto =====================
void mux_auto_step() {
  if (!g_mux_auto) return;
  uint32_t now = millis();
  if ((now - g_mux_last_switch) >= g_mux_auto_ms) {
    g_mux_last_switch = now;
    g_mux_state = (g_mux_state + 1) % 4;
    mux_apply(g_mux_state);
  }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(9600);
  analogReference(DEFAULT);

  pinMode(PIN_RELAY_CHARGE, OUTPUT);
  pinMode(PIN_RELAY_DISCHARGE, OUTPUT);
  pinMode(PIN_RELAY_POL_POS, OUTPUT);
  pinMode(PIN_RELAY_POL_NEG, OUTPUT);
  set_all_relays_low();

  pinMode(PIN_ADC_CHARGE, INPUT);
  pinMode(PIN_ADC_DISCHARGE_POS, INPUT);
  pinMode(PIN_ADC_DISCHARGE_NEG, INPUT);
  pinMode(PIN_ADC_CURRENT_SHUNT, INPUT);

  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  mux_apply(g_mux_state);

  Timer1.initialize(ADC_SAMPLE_PERIOD_US);
  Timer1.attachInterrupt(adc_timer_isr);

  Serial.println("READY");
  Serial.print("Config: Vcap_target=");
  Serial.print(g_target_vcap_V, 1);
  Serial.print("V E+%=");
  Serial.print(g_target_percent_pos, 1);
  Serial.print(" E-%=");
  Serial.println(g_target_percent_neg, 1);
}

// ===================== Status Report =====================
void report_status() {
  uint32_t now = millis();
  if (now - g_last_status_report < STATUS_REPORT_INTERVAL_MS) return;
  g_last_status_report = now;
  
  // Solo reportar si hay algo que reportar
  if (g_state == DEFIB_CHARGING) {
    uint16_t c = analogRead(PIN_ADC_CHARGE);
    float vcap = adc_to_vcap(c);
    Serial.print("[STATUS] CHARGING Vcap=");
    Serial.print(vcap, 2);
    Serial.println("V");
  }
}

// ===================== Loop =====================
void loop() {
  handle_serial();
  fsm_step();
  mux_auto_step();
  report_status();  // NUEVO
  delay(1);
}
