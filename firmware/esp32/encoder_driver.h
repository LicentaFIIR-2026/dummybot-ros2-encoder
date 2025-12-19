/*********************************************************************
 *  encoder_driver.h
 *  Driver pentru 4x encodere quadrature cu interrupts pe ESP32
 *  FL, FR, RL, RR - fiecare cu canale A și B
 *********************************************************************/

#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include "config.h"

// ============================================================================
// DEFINIRI ENCODERE (pentru indexare)
// ============================================================================

#define ENC_FL 0   // Front Left
#define ENC_FR 1   // Front Right
#define ENC_RL 2   // Rear Left
#define ENC_RR 3   // Rear Right

// Pentru compatibilitate cu differential drive logic
#define LEFT  10    // Alias pentru encoder stânga (media FL + RL)
#define RIGHT 11    // Alias pentru encoder dreapta (media FR + RR)

// ============================================================================
// FUNCȚII PUBLICE
// ============================================================================

void initEncoders();                    // Inițializare encodere cu interrupts
long readEncoder(int encoder);          // Citește tick count pentru un encoder (LEFT/RIGHT sau ENC_FL/FR/RL/RR)
void resetEncoder(int encoder);         // Resetează un encoder la 0
void resetEncoders();                   // Resetează toate encoderele la 0
void printEncoderCounts();              // Debug: afișează toate valorile encoderelor

// ============================================================================
// FUNCȚII ISR (Interrupt Service Routines) - nu se apelează direct
// ============================================================================

void IRAM_ATTR encoderFL_ISR();
void IRAM_ATTR encoderFR_ISR();
void IRAM_ATTR encoderRL_ISR();
void IRAM_ATTR encoderRR_ISR();

#endif // ENCODER_DRIVER_H
