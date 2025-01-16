#include "motor_interface.h"


double maxSpeed = 2000;
#define INPUT_A_FILTER 20

void initialize_motors(){
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_A_DIRECT_B_PWM);

  ConnectorM0.EnableRequest(true);
  ConnectorM1.EnableRequest(true);
  ConnectorM2.EnableRequest(true);
  ConnectorM3.EnableRequest(true);
}

bool CommandVelocity(MotorDriver motor, double commandedRPM) {
    if (commandedRPM > maxSpeed) {
      commandedRPM = maxSpeed * 0.95;
    }
    else if(commandedRPM < -maxSpeed){
      commandedRPM = -maxSpeed * 0.95;
    }
 
    // Check if an alert is currently preventing motion
    // if (motor.StatusReg().bit.AlertsPresent) {
    //     SerialPort.SendLine("Motor status: 'In Alert'. Move Canceled.");
    //     return false;
    // }
 
    // Change ClearPath's Input A state to change direction.
    // Note: this section of code was included so this commandVelocity function 
    // could be used to command negative (opposite direction) velocity. However the 
    // analog signal used by this example only commands positive velocities.
    if (commandedRPM >= 0) {
        motor.MotorInAState(false);
    }
    else {
        motor.MotorInAState(true);
    }
 
    // Delays to send the correct filtered direction.
    Delay_ms(20 + INPUT_A_FILTER);
 
    // Scale the velocity command to our duty cycle range.
    uint8_t dutyRequest = 1.0 * abs(commandedRPM) / maxSpeed * 255;
 
    // Command the move.
    motor.MotorInBDuty(dutyRequest);
 
    return true;
}

float get_motor_speed(MotorDriver motor)
{
  MotorDriver::HlfbStates hlfbState = motor.HlfbState();

  float motor_abs_speed = -1;

  if (hlfbState == MotorDriver::HLFB_HAS_MEASUREMENT) {
      float hlfbPercent = motor.HlfbPercent();
      motor_abs_speed = hlfbPercent * maxSpeed;
  }

  return motor_abs_speed;
}