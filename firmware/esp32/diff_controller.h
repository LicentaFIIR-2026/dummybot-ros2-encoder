/*********************************************************************
 *  diff_controller.h
 *  PID controller pentru 4 motoare independente
 *  Bazat pe ros_arduino_bridge adaptat pentru ESP32
 *********************************************************************/

#ifndef DIFF_CONTROLLER_H
#define DIFF_CONTROLLER_H

#include "config.h"
#include "encoder_driver.h"
#include "motor_driver_l298n.h"

// ============================================================================
// STRUCTURĂ PID pentru un motor
// ============================================================================

typedef struct {
  double TargetTicksPerFrame;    // Viteza țintă în ticks per frame
  long Encoder;                  // Valoarea curentă encoder
  long PrevEnc;                  // Valoarea anterioară encoder
  int PrevInput;                 // Input anterior (pentru derivative)
  int ITerm;                     // Termen integral acumulat
  long output;                   // Output PWM calculat
} SetPointInfo;

// ============================================================================
// VARIABILE GLOBALE PID (4 motoare independente)
// ============================================================================

extern SetPointInfo flPID, frPID, rlPID, rrPID;

// Parametri PID (se pot modifica prin comandă serial)
extern int Kp;  // Proportional gain
extern int Kd;  // Derivative gain
extern int Ki;  // Integral gain
extern int Ko;  // Output scaling factor

// Flag pentru mișcare
extern unsigned char moving;

// Factori de calibrare (se pot modifica prin comandă serial)
extern float calibration_fl;
extern float calibration_fr;
extern float calibration_rl;
extern float calibration_rr; 

// ============================================================================
// FUNCȚII PUBLICE
// ============================================================================

void resetPID();                         // Resetează structurile PID
void updatePID();                        // Actualizează PID și setează motoarele
void doPID(SetPointInfo *p);             // Calculează PID pentru un motor

#endif // DIFF_CONTROLLER_H