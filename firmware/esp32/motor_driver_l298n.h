/*********************************************************************
 *  motor_driver_l298n.h
 *  Driver pentru 2x L298N (4 motoare DC) - CONTROL INDEPENDENT
 *  Bazat pe ros_arduino_bridge adaptat pentru ESP32
 *********************************************************************/

#ifndef MOTOR_DRIVER_L298N_H
#define MOTOR_DRIVER_L298N_H

#include "config.h"

// ============================================================================
// DEFINIRI MOTOARE INDIVIDUALE
// ============================================================================

#define MOTOR_FL 0   // Front Left
#define MOTOR_FR 1   // Front Right
#define MOTOR_RL 2   // Rear Left
#define MOTOR_RR 3   // Rear Right

// ============================================================================
// FUNCȚII PUBLICE
// ============================================================================

void initMotorController();                        // Inițializare PWM și pini
void setMotorSpeed(int motor, int spd);            // Setează viteza unui motor individual
void setMotorSpeeds(int flSpeed, int frSpeed, int rlSpeed, int rrSpeed); // Setează toate cele 4 motoare
void stopMotors();                                  // Oprește toate motoarele

#endif // MOTOR_DRIVER_L298N_H