/*********************************************************************
 *  diff_controller.ino
 *  Implementare PID pentru 4 motoare independente
 *********************************************************************/

#include "diff_controller.h"

// ============================================================================
// VARIABILE GLOBALE PID
// ============================================================================

SetPointInfo flPID, frPID, rlPID, rrPID;

int Kp = PID_KP;
int Kd = PID_KD;
int Ki = PID_KI;
int Ko = PID_KO;

unsigned char moving = 0;

// Factori de calibrare pentru fiecare motor
float calibration_fl = CALIBRATION_FACTOR_FL;
float calibration_fr = CALIBRATION_FACTOR_FR;
float calibration_rl = CALIBRATION_FACTOR_RL;
float calibration_rr = CALIBRATION_FACTOR_RR;

// ============================================================================
// RESETARE PID
// ============================================================================

void resetPID() {
  flPID.TargetTicksPerFrame = 0.0;
  flPID.Encoder = readEncoder(ENC_FL);
  flPID.PrevEnc = flPID.Encoder;
  flPID.output = 0;
  flPID.PrevInput = 0;
  flPID.ITerm = 0;
  
  frPID.TargetTicksPerFrame = 0.0;
  frPID.Encoder = readEncoder(ENC_FR);
  frPID.PrevEnc = frPID.Encoder;
  frPID.output = 0;
  frPID.PrevInput = 0;
  frPID.ITerm = 0;
  
  rlPID.TargetTicksPerFrame = 0.0;
  rlPID.Encoder = readEncoder(ENC_RL);
  rlPID.PrevEnc = rlPID.Encoder;
  rlPID.output = 0;
  rlPID.PrevInput = 0;
  rlPID.ITerm = 0;
  
  rrPID.TargetTicksPerFrame = 0.0;
  rrPID.Encoder = readEncoder(ENC_RR);
  rrPID.PrevEnc = rrPID.Encoder;
  rrPID.output = 0;
  rrPID.PrevInput = 0;
  rrPID.ITerm = 0;
}

// ============================================================================
// ACTUALIZARE PID (rulează la fiecare PID_INTERVAL)
// ============================================================================

void updatePID() {
  // Citește valorile curente ale encoderelor
  flPID.Encoder = readEncoder(ENC_FL);
  frPID.Encoder = readEncoder(ENC_FR);
  rlPID.Encoder = readEncoder(ENC_RL);
  rrPID.Encoder = readEncoder(ENC_RR);
  
  if (moving) {
    // Calculează PID pentru fiecare motor
    doPID(&flPID);
    doPID(&frPID);
    doPID(&rlPID);
    doPID(&rrPID);
    
    // Aplică factorii de calibrare
    long fl_PWM = flPID.output * calibration_fl;
    long fr_PWM = frPID.output * calibration_fr;
    long rl_PWM = rlPID.output * calibration_rl;
    long rr_PWM = rrPID.output * calibration_rr;
    
    // Setează motoarele individuale cu calibrare aplicată
    setMotorSpeed(MOTOR_FL, fl_PWM);
    setMotorSpeed(MOTOR_FR, fr_PWM);
    setMotorSpeed(MOTOR_RL, rl_PWM);
    setMotorSpeed(MOTOR_RR, rr_PWM);
    
  } else {
    // Dacă nu se mișcă, resetează outputurile
    flPID.output = 0;
    frPID.output = 0;
    rlPID.output = 0;
    rrPID.output = 0;
    
    // Actualizează encoderele anterioare
    flPID.PrevEnc = flPID.Encoder;
    frPID.PrevEnc = frPID.Encoder;
    rlPID.PrevEnc = rlPID.Encoder;
    rrPID.PrevEnc = rrPID.Encoder;
  }
}

// ============================================================================
// CALCUL PID pentru un motor
// ============================================================================

void doPID(SetPointInfo *p) {
  // Calculează câți ticks s-au mișcat de la ultima actualizare
  long Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  
  // Calculează termenul derivative (D)
  int output = (Kp * Perror - Kd * (p->Encoder - p->PrevEnc) + p->ITerm) / Ko;
  
  // Salvează encoder anterior
  p->PrevEnc = p->Encoder;
  
  // Limitează output la PWM max
  if (output > MAX_PWM) {
    output = MAX_PWM;
  } else if (output < -MAX_PWM) {
    output = -MAX_PWM;
  }
  
  // Actualizează termenul integral (I) doar dacă nu suntem la limită
  if (output != MAX_PWM && output != -MAX_PWM) {
    p->ITerm += Ki * Perror;
  }
  
  p->output = output;
}