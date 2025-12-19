/*********************************************************************
 *  commands.h
 *  Protocol serial pentru comunicare cu ROS2
 *  Bazat pe ros_arduino_bridge și adaptat pentru ESP32
 *********************************************************************/

#ifndef COMMANDS_H
#define COMMANDS_H

// ============================================================================
// COMENZI SERIAL (caractere simple)
// ============================================================================

#define GET_BAUDRATE   'b'    // Returnează baudrate-ul curent
#define READ_ENCODERS  'e'    // Citește valorile encoderelor (4 motoare)
#define MOTOR_SPEEDS   'm'    // Setează viteze cu PID (FL, FR, RL, RR)
#define MOTOR_RAW_PWM  'o'    // Setează PWM direct fără PID (FL, FR, RL, RR)
#define RESET_ENCODERS 'r'    // Resetează contoarele encoderelor la 0
#define UPDATE_PID     'u'    // Actualizează parametrii PID (Kp, Kd, Ki, Ko)
#define GET_PID        'p'    // Returnează parametrii PID actuali
#define AUTO_STOP      's'    // Oprește automat motoarele (safety)
#define TEST_MOTORS    't'    // Testează motoarele individual
#define SET_CALIBRATION 'c'   // Setează factori de calibrare pentru 4 motoare
#define RESET_CALIBRATION 'C' // Resetează calibrarea la valorile default

// ============================================================================
// PROTOCOL DETALIAT
// ============================================================================

/*
 * COMANDĂ: 'b' - GET_BAUDRATE
 * Request:  b
 * Response: 115200
 * 
 * COMANDĂ: 'e' - READ_ENCODERS
 * Request:  e
 * Response: FL FR RL RR (4 valori long separate prin spații)
 * Exemplu:  1234 -567 890 -123
 * 
 * COMANDĂ: 'm' - MOTOR_SPEEDS (cu PID) - 4 MOTOARE INDEPENDENTE
 * Request:  m FL FR RL RR
 * Exemplu:  m 50 50 50 50      (înainte la 50 ticks/frame pe toate)
 *           m -30 -30 -30 -30  (înapoi la 30 ticks/frame)
 *           m 40 -40 40 -40    (rotire pe loc)
 *           m 50 30 50 30      (viraj la dreapta)
 * Response: OK
 * 
 * COMANDĂ: 'o' - MOTOR_RAW_PWM (fără PID, direct PWM) - 4 MOTOARE INDEPENDENTE
 * Request:  o FL FR RL RR
 * Exemplu:  o 150 150 150 150  (PWM 150 pe toate motoarele)
 *           o -100 100 -100 100 (rotire cu PWM 100)
 * Response: OK
 * 
 * COMANDĂ: 'r' - RESET_ENCODERS
 * Request:  r
 * Response: OK
 * 
 * COMANDĂ: 'u' - UPDATE_PID
 * Request:  u Kp Kd Ki Ko
 * Exemplu:  u 60 20 0 15
 * Response: OK
 * 
 * COMANDĂ: 'p' - GET_PID
 * Request:  p
 * Response: Kp:60 Kd:20 Ki:0 Ko:15
 * 
 * COMANDĂ: 's' - AUTO_STOP
 * Request:  s
 * Response: OK (oprește toate motoarele)
 */

#endif // COMMANDS_H