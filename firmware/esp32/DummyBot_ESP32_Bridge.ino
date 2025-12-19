/*********************************************************************
 *  DummyBot_ESP32_Bridge.ino
 *  Bridge între ROS2 și ESP32 pentru DummyBot
 *  
 *  Hardware:
 *  - ESP32 DevKit
 *  - 2x L298N motor drivers (4 motoare DC)
 *  - 4x Encodere quadrature (1760 ticks/rev)
 *  
 *  Comunicare:
 *  - Serial: 115200 baud
 *  - Protocol: comenzi ASCII (compatibil ros_arduino_bridge)
 *  
 *  Autor: Adaptat pentru DummyBot
 *  Bazat pe: ros_arduino_bridge + ROS_ESP32_Bridge
 *********************************************************************/

#include "config.h"
#include "commands.h"
#include "motor_driver_l298n.h"
#include "encoder_driver.h"
#include "diff_controller.h"
#include <EEPROM.h>

// ============================================================================
// VARIABILE GLOBALE
// ============================================================================

// Timing pentru PID loop
unsigned long nextPID = 0;

// Auto-stop safety (oprește motoarele dacă nu primește comenzi)
unsigned long lastCmdTime = 0;
bool autoStopEnabled = true;

// Buffer pentru comenzi seriale
const int MAX_CMD_LENGTH = 64;
char cmdBuffer[MAX_CMD_LENGTH];
int cmdIndex = 0;

// ============================================================================
// FUNCȚII PENTRU STOCARE CALIBRARE ÎN EEPROM
// ============================================================================

void saveCalibrationToEEPROM() {
  EEPROM.writeUShort(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
  EEPROM.writeFloat(EEPROM_ADDR_CAL_FL, calibration_fl);
  EEPROM.writeFloat(EEPROM_ADDR_CAL_FR, calibration_fr);
  EEPROM.writeFloat(EEPROM_ADDR_CAL_RL, calibration_rl);
  EEPROM.writeFloat(EEPROM_ADDR_CAL_RR, calibration_rr);
  EEPROM.commit();
  Serial.println("Calibration saved to EEPROM");
}

void loadCalibrationFromEEPROM() {
  uint16_t magic = EEPROM.readUShort(EEPROM_ADDR_MAGIC);
  
  if (magic == EEPROM_MAGIC) {
    calibration_fl = EEPROM.readFloat(EEPROM_ADDR_CAL_FL);
    calibration_fr = EEPROM.readFloat(EEPROM_ADDR_CAL_FR);
    calibration_rl = EEPROM.readFloat(EEPROM_ADDR_CAL_RL);
    calibration_rr = EEPROM.readFloat(EEPROM_ADDR_CAL_RR);
    
    Serial.println("Calibration loaded from EEPROM:");
    Serial.print("  FL: "); Serial.println(calibration_fl, 3);
    Serial.print("  FR: "); Serial.println(calibration_fr, 3);
    Serial.print("  RL: "); Serial.println(calibration_rl, 3);
    Serial.print("  RR: "); Serial.println(calibration_rr, 3);
  } else {
    Serial.println("No valid calibration in EEPROM, using defaults");
    calibration_fl = CALIBRATION_FACTOR_FL;
    calibration_fr = CALIBRATION_FACTOR_FR;
    calibration_rl = CALIBRATION_FACTOR_RL;
    calibration_rr = CALIBRATION_FACTOR_RR;
    saveCalibrationToEEPROM();
  }
}

void resetCalibrationToDefaults() {
  calibration_fl = CALIBRATION_FACTOR_FL;
  calibration_fr = CALIBRATION_FACTOR_FR;
  calibration_rl = CALIBRATION_FACTOR_RL;
  calibration_rr = CALIBRATION_FACTOR_RR;
  saveCalibrationToEEPROM();
  Serial.println("Calibration reset to defaults");
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // --- Inițializare Serial ---
  Serial.begin(BAUDRATE);
  while (!Serial) {
    ; // Așteaptă conectarea serial (pentru debug)
  }

  // --- Inițializare EEPROM ---
  EEPROM.begin(EEPROM_SIZE);
  loadCalibrationFromEEPROM(); 
  
  Serial.println("===========================================");
  Serial.println("  DummyBot ESP32 Bridge");
  Serial.println("  Ready for ROS2 communication");
  Serial.println("  4 Independent Motors Control");
  Serial.println("===========================================");

  // --- Inițializare Hardware ---
  initMotorController();
  initEncoders();
  
  // --- Inițializare PID ---
  resetPID();
  
  // --- Setează timer pentru PID ---
  nextPID = millis() + PID_INTERVAL;
  
  Serial.print("Baudrate: ");
  Serial.println(BAUDRATE);
  Serial.print("PID Rate: ");
  Serial.print(PID_RATE);
  Serial.println(" Hz");
  Serial.print("Ticks per revolution: ");
  Serial.println(TICKS_PER_REV);
  Serial.println("System ready!");
  Serial.println();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // --- Procesare comenzi seriale ---
  while (Serial.available() > 0) {
    processSerialCommand();
  }
  
  // --- Actualizare PID la interval regulat ---
  if (millis() >= nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // --- Auto-stop safety ---
  if (autoStopEnabled && moving && (millis() - lastCmdTime > AUTO_STOP_INTERVAL)) {
    moving = 0;
    stopMotors();
    resetPID();
    Serial.println("AUTO-STOP: No command received");
  }
}

// ============================================================================
// PROCESARE COMENZI SERIALE
// ============================================================================

void processSerialCommand() {
  char c = Serial.read();
  
  // Adaugă caracterul în buffer
  if (c == '\n' || c == '\r') {
    // Sfârșit de comandă
    if (cmdIndex > 0) {
      cmdBuffer[cmdIndex] = '\0';  // Termină string-ul
      executeCommand();
      cmdIndex = 0;  // Resetează buffer
    }
  } else if (cmdIndex < MAX_CMD_LENGTH - 1) {
    cmdBuffer[cmdIndex++] = c;
  }
}

// ============================================================================
// EXECUTARE COMANDĂ
// ============================================================================

void executeCommand() {
  char cmd = cmdBuffer[0];
  
  switch(cmd) {
    
    // --- GET BAUDRATE ---
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    
    // --- READ ENCODERS ---
    case READ_ENCODERS:
      {
        long fl = readEncoder(ENC_FL);
        long fr = readEncoder(ENC_FR);
        long rl = readEncoder(ENC_RL);
        long rr = readEncoder(ENC_RR);
        
        Serial.print(fl);
        Serial.print(" ");
        Serial.print(fr);
        Serial.print(" ");
        Serial.print(rl);
        Serial.print(" ");
        Serial.println(rr);
      }
      break;
    
    // --- MOTOR SPEEDS (cu PID) - 4 MOTOARE INDEPENDENTE ---
    case MOTOR_SPEEDS:
      {
        int flSpeed, frSpeed, rlSpeed, rrSpeed;
        if (sscanf(cmdBuffer, "m %d %d %d %d", &flSpeed, &frSpeed, &rlSpeed, &rrSpeed) == 4) {
          lastCmdTime = millis();
          moving = 1;
          
          flPID.TargetTicksPerFrame = flSpeed;
          frPID.TargetTicksPerFrame = frSpeed;
          rlPID.TargetTicksPerFrame = rlSpeed;
          rrPID.TargetTicksPerFrame = rrSpeed;
          
          Serial.println("OK");
        } else {
          Serial.println("ERROR: Invalid motor speeds format (need 4 values)");
        }
      }
      break;
    
    // --- MOTOR RAW PWM (fără PID) - 4 MOTOARE INDEPENDENTE ---
    case MOTOR_RAW_PWM:
      {
        int flPWM, frPWM, rlPWM, rrPWM;
        if (sscanf(cmdBuffer, "o %d %d %d %d", &flPWM, &frPWM, &rlPWM, &rrPWM) == 4) {
          lastCmdTime = millis();
          moving = 0;  // Dezactivează PID când folosești PWM raw
          
          setMotorSpeeds(flPWM, frPWM, rlPWM, rrPWM);
          
          Serial.println("OK");
        } else {
          Serial.println("ERROR: Invalid PWM format (need 4 values)");
        }
      }
      break;
    
    // --- RESET ENCODERS ---
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    
    // --- UPDATE PID PARAMETERS ---
    case UPDATE_PID:
      {
        int kp, kd, ki, ko;
        if (sscanf(cmdBuffer, "u %d %d %d %d", &kp, &kd, &ki, &ko) == 4) {
          Kp = kp;
          Kd = kd;
          Ki = ki;
          Ko = ko;
          
          Serial.println("OK");
          Serial.print("PID updated - Kp:");
          Serial.print(Kp);
          Serial.print(" Kd:");
          Serial.print(Kd);
          Serial.print(" Ki:");
          Serial.print(Ki);
          Serial.print(" Ko:");
          Serial.println(Ko);
        } else {
          Serial.println("ERROR: Invalid PID format");
        }
      }
      break;
    
    // --- GET PID PARAMETERS ---
    case GET_PID:
      Serial.print("Kp:");
      Serial.print(Kp);
      Serial.print(" Kd:");
      Serial.print(Kd);
      Serial.print(" Ki:");
      Serial.print(Ki);
      Serial.print(" Ko:");
      Serial.println(Ko);
      break;

    // --- SET CALIBRATION FACTORS ---
    case SET_CALIBRATION:
      {
        float cal_fl, cal_fr, cal_rl, cal_rr;
        if (sscanf(cmdBuffer, "c %f %f %f %f", &cal_fl, &cal_fr, &cal_rl, &cal_rr) == 4) {
          calibration_fl = cal_fl;
          calibration_fr = cal_fr;
          calibration_rl = cal_rl;
          calibration_rr = cal_rr;
          saveCalibrationToEEPROM();
          
          Serial.println("OK");
          Serial.print("Calibration - FL:");
          Serial.print(calibration_fl, 3);
          Serial.print(" FR:");
          Serial.print(calibration_fr, 3);
          Serial.print(" RL:");
          Serial.print(calibration_rl, 3);
          Serial.print(" RR:");
          Serial.println(calibration_rr, 3);
        } else {
          Serial.println("ERROR: Invalid calibration format");
        }
      }
      break;
    
    // --- RESET CALIBRATION TO DEFAULTS ---
    case RESET_CALIBRATION:
      resetCalibrationToDefaults();
      Serial.println("OK");
      break;  
    
    // --- AUTO STOP ---
    case AUTO_STOP:
      moving = 0;
      stopMotors();
      resetPID();
      Serial.println("OK");
      break;

    // --- TEST MOTORS (pentru debugging) ---
    case TEST_MOTORS:
      {
        Serial.println("Starting motor test sequence...");
        Serial.println("Each motor will run for 2 seconds");
        
        // Test FL
        Serial.println("Testing MOTOR_FL (Front Left)...");
        setMotorSpeed(MOTOR_FL, 150);
        delay(2000);
        setMotorSpeed(MOTOR_FL, 0);
        delay(500);
        
        // Test FR
        Serial.println("Testing MOTOR_FR (Front Right)...");
        setMotorSpeed(MOTOR_FR, 150);
        delay(2000);
        setMotorSpeed(MOTOR_FR, 0);
        delay(500);
        
        // Test RL
        Serial.println("Testing MOTOR_RL (Rear Left)...");
        setMotorSpeed(MOTOR_RL, 150);
        delay(2000);
        setMotorSpeed(MOTOR_RL, 0);
        delay(500);
        
        // Test RR
        Serial.println("Testing MOTOR_RR (Rear Right)...");
        setMotorSpeed(MOTOR_RR, 150);
        delay(2000);
        setMotorSpeed(MOTOR_RR, 0);
        delay(500);
        
        Serial.println("Motor test complete!");
        Serial.println("OK");
      }
      break;  
    
    // --- COMANDĂ NECUNOSCUTĂ ---
    default:
      Serial.print("ERROR: Unknown command '");
      Serial.print(cmd);
      Serial.println("'");
      break;
  }
}