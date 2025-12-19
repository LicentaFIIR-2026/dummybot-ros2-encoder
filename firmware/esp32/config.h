/*********************************************************************
 *  config.h - Configurare completă DummyBot
 *  Toate parametrii hardware și software într-un singur loc
 *********************************************************************/

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// CONFIGURARE HARDWARE - MOTOARE ȘI ENCODERE
// ============================================================================

// --- PINI L298N DRIVER 1 (STÂNGA: FL + RL) ---
#define ENA_L 13      // PWM pentru motoarele stânga
#define IN1_L 12      // Direcție FL Motor A
#define IN2_L 14      // Direcție FL Motor A
#define IN3_L 27      // Direcție RL Motor B
#define IN4_L 26      // Direcție RL Motor B
#define ENB_L 25      // PWM pentru motoarele stânga

// --- PINI L298N DRIVER 2 (DREAPTA: FR + RR) ---
#define ENA_R 15      // PWM pentru FR Motor A (ERA 19, ACUM 15)
#define IN1_R 2       // Direcție FR Motor A (ERA 18, ACUM 2)
#define IN2_R 4       // Direcție FR Motor A (ERA 5, ACUM 4)
#define IN3_R 18      // Direcție RR Motor B (ERA 4, ACUM 18)
#define IN4_R 5       // Direcție RR Motor B (ERA 2, ACUM 5)
#define ENB_R 19      // PWM pentru RR Motor B (ERA 15, ACUM 19)

// --- PINI ENCODERE (pe ESP32 același cu motoarele) ---
// Front Left (FL)
#define ENCODER_FL_A 32
#define ENCODER_FL_B 35

// Front Right (FR)
#define ENCODER_FR_A 21
#define ENCODER_FR_B 33

// Rear Left (RL)
#define ENCODER_RL_A 16
#define ENCODER_RL_B 17

// Rear Right (RR)
#define ENCODER_RR_A 22
#define ENCODER_RR_B 23

// ============================================================================
// PARAMETRI ROȚI ȘI ROBOT
// ============================================================================

// --- Specificații Roți ---
#define WHEEL_DIAMETER 0.065        // Diametru roată: 65mm = 0.065m
#define WHEEL_RADIUS 0.0325         // Rază roată: 32.5mm = 0.0325m
#define WHEEL_CIRCUMFERENCE 0.2042  // Circumferință: 2*PI*r = 0.2042m

// --- Specificații Encodere ---
#define ENCODER_PPR 11              // Pulses per revolution (pe un canal)
#define GEAR_RATIO 40               // Gear ratio: 40:1
#define ENCODER_CPR 44              // Counts per motor revolution (quadrature: 11*4)
#define TICKS_PER_REV 1760          // Ticks per wheel revolution: 44*40 = 1760
#define METERS_PER_TICK 0.000116    // Distanță per tick: 0.2042/1760 = 0.116mm

// --- Dimensiuni Robot ---
#define WHEEL_TRACK 0.3863          // Lx: distanță stânga-dreapta (center-to-center)
#define WHEELBASE 0.37232           // Ly: distanță față-spate (center-to-center)

// ============================================================================
// PARAMETRI CONTROL ȘI PID
// ============================================================================

// --- PWM Settings ---
#define PWM_FREQUENCY 20000         // 20kHz PWM frequency
#define PWM_RESOLUTION 8            // 8-bit resolution (0-255)
#define MAX_PWM 255                 // Maxim PWM value

// --- PID Parameters (se pot ajusta) ---
#define PID_KP 60                   // Proportional gain
#define PID_KD 20                   // Derivative gain
#define PID_KI 0                    // Integral gain
#define PID_KO 15                   // Output scaling factor

// --- PID Loop Rate ---
#define PID_RATE 30                 // Hz - cât de des rulează PID (30Hz = la 33ms)
#define PID_INTERVAL (1000 / PID_RATE)  // Interval în milisecunde

// --- Auto-Stop Safety ---
#define AUTO_STOP_INTERVAL 2000     // Oprește motoarele dacă nu primește comenzi 2 secunde

// ============================================================================
// COMUNICARE SERIAL
// ============================================================================

#define BAUDRATE 115200             // Serial baud rate

// ============================================================================
// CALIBRARE (factori de corecție pentru diferențe între motoare)
// ============================================================================

#define CALIBRATION_FACTOR_FL 1.0   // Factor calibrare Front Left
#define CALIBRATION_FACTOR_FR 1.0   // Factor calibrare Front Right
#define CALIBRATION_FACTOR_RL 1.0   // Factor calibrare Rear Left
#define CALIBRATION_FACTOR_RR 1.0   // Factor calibrare Rear Right

// ============================================================================
// STOCARE CALIBRARE ÎN EEPROM
// ============================================================================

#define EEPROM_SIZE 64              // Dimensiune EEPROM (bytes)
#define EEPROM_MAGIC 0xABCD         // Valoare magică pentru validare
#define EEPROM_ADDR_MAGIC 0         // Adresa pentru magic number
#define EEPROM_ADDR_CAL_FL 2        // Adresa pentru calibration FL
#define EEPROM_ADDR_CAL_FR 6        // Adresa pentru calibration FR
#define EEPROM_ADDR_CAL_RL 10       // Adresa pentru calibration RL
#define EEPROM_ADDR_CAL_RR 14       // Adresa pentru calibration RR

// ============================================================================
// DEBUG FLAGS
// ============================================================================

//#define DEBUG_ENCODERS            // Decomentează pentru debug encodere
//#define DEBUG_PID                 // Decomentează pentru debug PID
//#define DEBUG_MOTORS              // Decomentează pentru debug motoare

#endif // CONFIG_H
