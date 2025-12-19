/*********************************************************************
 *  motor_driver_l298n.ino
 *  Implementare driver pentru 2x L298N (4 motoare) - CONTROL INDEPENDENT
 *********************************************************************/

#include "motor_driver_l298n.h"

// ============================================================================
// INIȚIALIZARE MOTOR CONTROLLER
// ============================================================================

void initMotorController() {
  // --- Configurare pini L298N STÂNGA ---
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN3_L, OUTPUT);
  pinMode(IN4_L, OUTPUT);
  
  // --- Configurare pini L298N DREAPTA ---
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(IN3_R, OUTPUT);
  pinMode(IN4_R, OUTPUT);
  
  // --- Configurare PWM pentru ESP32 (compatibil Core 2.x și 3.x) ---
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    // ESP32 Core 3.x - API nou
    ledcAttach(ENA_L, PWM_FREQUENCY, PWM_RESOLUTION);  // FL motor
    ledcAttach(ENB_L, PWM_FREQUENCY, PWM_RESOLUTION);  // RL motor
    ledcAttach(ENA_R, PWM_FREQUENCY, PWM_RESOLUTION);  // FR motor
    ledcAttach(ENB_R, PWM_FREQUENCY, PWM_RESOLUTION);  // RR motor
  #else
    // ESP32 Core 2.x - API vechi
    ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);  // Channel pentru FL
    ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION);  // Channel pentru RL
    ledcSetup(2, PWM_FREQUENCY, PWM_RESOLUTION);  // Channel pentru FR
    ledcSetup(3, PWM_FREQUENCY, PWM_RESOLUTION);  // Channel pentru RR
    
    ledcAttachPin(ENA_L, 0);  // FL motor
    ledcAttachPin(ENB_L, 1);  // RL motor
    ledcAttachPin(ENA_R, 2);  // FR motor
    ledcAttachPin(ENB_R, 3);  // RR motor
  #endif
  
  // --- Oprește toate motoarele ---
  stopMotors();
  
  Serial.println("Motor controller initialized (4 independent motors)");
  Serial.println("Motor mapping:");
  Serial.println("  MOTOR_FL (0) -> ENA_L (pin 13) -> IN1_L, IN2_L");
  Serial.println("  MOTOR_FR (1) -> ENA_R (pin 19) -> IN1_R, IN2_R");
  Serial.println("  MOTOR_RL (2) -> ENB_L (pin 25) -> IN3_L, IN4_L");
  Serial.println("  MOTOR_RR (3) -> ENB_R (pin 15) -> IN3_R, IN4_R");
}

// ============================================================================
// SETARE VITEZĂ MOTOR INDIVIDUAL
// ============================================================================

void setMotorSpeed(int motor, int spd) {
  int pwmValue = abs(spd);
  if (pwmValue > MAX_PWM) pwmValue = MAX_PWM;
  
  switch(motor) {
    // --- FRONT LEFT (FL) - Motor A pe Driver Stânga ---
    case MOTOR_FL:
      if (spd > 0) {
        digitalWrite(IN1_L, HIGH);
        digitalWrite(IN2_L, LOW);
      } else if (spd < 0) {
        digitalWrite(IN1_L, LOW);
        digitalWrite(IN2_L, HIGH);
      } else {
        digitalWrite(IN1_L, LOW);
        digitalWrite(IN2_L, LOW);
      }
      #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(ENA_L, pwmValue);
      #else
        ledcWrite(0, pwmValue);  // Channel 0
      #endif
      break;
    
    // --- FRONT RIGHT (FR) - Motor A pe Driver Dreapta ---
    case MOTOR_FR:
      if (spd > 0) {
        digitalWrite(IN1_R, HIGH);
        digitalWrite(IN2_R, LOW);
      } else if (spd < 0) {
        digitalWrite(IN1_R, LOW);
        digitalWrite(IN2_R, HIGH);
      } else {
        digitalWrite(IN1_R, LOW);
        digitalWrite(IN2_R, LOW);
      }
      #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(ENA_R, pwmValue);
      #else
        ledcWrite(2, pwmValue);  // Channel 2
      #endif
      break;
    
    // --- REAR LEFT (RL) - Motor B pe Driver Stânga ---
    case MOTOR_RL:
      if (spd > 0) {
        digitalWrite(IN3_L, HIGH);
        digitalWrite(IN4_L, LOW);
      } else if (spd < 0) {
        digitalWrite(IN3_L, LOW);
        digitalWrite(IN4_L, HIGH);
      } else {
        digitalWrite(IN3_L, LOW);
        digitalWrite(IN4_L, LOW);
      }
      #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(ENB_L, pwmValue);
      #else
        ledcWrite(1, pwmValue);  // Channel 1
      #endif
      break;
    
    // --- REAR RIGHT (RR) - Motor B pe Driver Dreapta ---
    case MOTOR_RR:
      if (spd > 0) {
        digitalWrite(IN3_R, HIGH);
        digitalWrite(IN4_R, LOW);
      } else if (spd < 0) {
        digitalWrite(IN3_R, LOW);
        digitalWrite(IN4_R, HIGH);
      } else {
        digitalWrite(IN3_R, LOW);
        digitalWrite(IN4_R, LOW);
      }
      #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(ENB_R, pwmValue);
      #else
        ledcWrite(3, pwmValue);  // Channel 3
      #endif
      break;
  }
}

// ============================================================================
// SETARE TOATE MOTOARELE (4 valori)
// ============================================================================

void setMotorSpeeds(int flSpeed, int frSpeed, int rlSpeed, int rrSpeed) {
  setMotorSpeed(MOTOR_FL, flSpeed);
  setMotorSpeed(MOTOR_FR, frSpeed);
  setMotorSpeed(MOTOR_RL, rlSpeed);
  setMotorSpeed(MOTOR_RR, rrSpeed);
}

// ============================================================================
// OPRIRE TOATE MOTOARELE
// ============================================================================

void stopMotors() {
  setMotorSpeed(MOTOR_FL, 0);
  setMotorSpeed(MOTOR_FR, 0);
  setMotorSpeed(MOTOR_RL, 0);
  setMotorSpeed(MOTOR_RR, 0);
}