/*********************************************************************
 *  encoder_driver.ino
 *  Implementare citire encodere quadrature cu interrupts
 *  4 encodere: FL, FR, RL, RR - fiecare 1760 ticks/revolution
 *********************************************************************/

#include "encoder_driver.h"

// ============================================================================
// VARIABILE GLOBALE pentru TICK COUNTS (volatile pentru ISR)
// ============================================================================

volatile long encoder_count_FL = 0;
volatile long encoder_count_FR = 0;
volatile long encoder_count_RL = 0;
volatile long encoder_count_RR = 0;

// ============================================================================
// INIȚIALIZARE ENCODERE
// ============================================================================

void initEncoders() {
  // --- Setup pini ca INPUT ---
  pinMode(ENCODER_FL_A, INPUT);
  pinMode(ENCODER_FL_B, INPUT);
  pinMode(ENCODER_FR_A, INPUT);
  pinMode(ENCODER_FR_B, INPUT);
  pinMode(ENCODER_RL_A, INPUT);
  pinMode(ENCODER_RL_B, INPUT);
  pinMode(ENCODER_RR_A, INPUT);
  pinMode(ENCODER_RR_B, INPUT);

  // --- Attach interrupts pe canalul A (CHANGE = orice tranziție) ---
  attachInterrupt(digitalPinToInterrupt(ENCODER_FL_A), encoderFL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_FR_A), encoderFR_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RL_A), encoderRL_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RR_A), encoderRR_ISR, CHANGE);

  // --- Resetează contoarele ---
  resetEncoders();

  #ifdef DEBUG_ENCODERS
  Serial.println("Encoders initialized (4x quadrature with interrupts)");
  #endif
}

// ============================================================================
// ISR - FRONT LEFT ENCODER
// ============================================================================

void IRAM_ATTR encoderFL_ISR() {
  // Citește starea ambelor canale
  int A = digitalRead(ENCODER_FL_A);
  int B = digitalRead(ENCODER_FL_B);
  
  // Quadrature decoding: A XOR B = direcția
  if (A == B) {
    encoder_count_FL--;  // Înainte
  } else {
    encoder_count_FL++;  // Înapoi
  }
}

// ============================================================================
// ISR - FRONT RIGHT ENCODER
// ============================================================================

void IRAM_ATTR encoderFR_ISR() {
  int A = digitalRead(ENCODER_FR_A);
  int B = digitalRead(ENCODER_FR_B);
  
  if (A == B) {
    encoder_count_FR--;
  } else {
    encoder_count_FR++;
  }
}

// ============================================================================
// ISR - REAR LEFT ENCODER
// ============================================================================

void IRAM_ATTR encoderRL_ISR() {
  int A = digitalRead(ENCODER_RL_A);
  int B = digitalRead(ENCODER_RL_B);
  
  if (A == B) {
    encoder_count_RL--;
  } else {
    encoder_count_RL++;
  }
}

// ============================================================================
// ISR - REAR RIGHT ENCODER
// ============================================================================

void IRAM_ATTR encoderRR_ISR() {
  int A = digitalRead(ENCODER_RR_A);
  int B = digitalRead(ENCODER_RR_B);
  
  if (A == B) {
    encoder_count_RR--;
  } else {
    encoder_count_RR++;
  }
}

// ============================================================================
// CITIRE ENCODER (individual sau medie pentru LEFT/RIGHT)
// ============================================================================

long readEncoder(int encoder) {
  long count = 0;
  
  noInterrupts();  // Dezactivează interrupts temporar pentru citire atomică
  
  switch(encoder) {
    case ENC_FL:
      count = encoder_count_FL;
      break;
    case ENC_FR:
      count = encoder_count_FR;
      break;
    case ENC_RL:
      count = encoder_count_RL;
      break;
    case ENC_RR:
      count = encoder_count_RR;
      break;
    case LEFT:
      // Media encoderelor stânga (FL + RL) / 2
      count = (encoder_count_FL + encoder_count_RL) / 2;
      break;
    case RIGHT:
      // Media encoderelor dreapta (FR + RR) / 2
      count = (encoder_count_FR + encoder_count_RR) / 2;
      break;
    default:
      count = 0;
  }
  
  interrupts();  // Reactivează interrupts
  
  return count;
}

// ============================================================================
// RESETARE UN ENCODER
// ============================================================================

void resetEncoder(int encoder) {
  noInterrupts();
  
  switch(encoder) {
    case ENC_FL:
      encoder_count_FL = 0;
      break;
    case ENC_FR:
      encoder_count_FR = 0;
      break;
    case ENC_RL:
      encoder_count_RL = 0;
      break;
    case ENC_RR:
      encoder_count_RR = 0;
      break;
    case LEFT:
      encoder_count_FL = 0;
      encoder_count_RL = 0;
      break;
    case RIGHT:
      encoder_count_FR = 0;
      encoder_count_RR = 0;
      break;
  }
  
  interrupts();
}

// ============================================================================
// RESETARE TOATE ENCODERELE
// ============================================================================

void resetEncoders() {
  noInterrupts();
  
  encoder_count_FL = 0;
  encoder_count_FR = 0;
  encoder_count_RL = 0;
  encoder_count_RR = 0;
  
  interrupts();
  
  #ifdef DEBUG_ENCODERS
  Serial.println("All encoders reset to 0");
  #endif
}

// ============================================================================
// DEBUG - AFIȘARE VALORI ENCODERE
// ============================================================================

void printEncoderCounts() {
  noInterrupts();
  long fl = encoder_count_FL;
  long fr = encoder_count_FR;
  long rl = encoder_count_RL;
  long rr = encoder_count_RR;
  interrupts();
  
  Serial.print("Encoders FL: ");
  Serial.print(fl);
  Serial.print(" FR: ");
  Serial.print(fr);
  Serial.print(" RL: ");
  Serial.print(rl);
  Serial.print(" RR: ");
  Serial.println(rr);
}
