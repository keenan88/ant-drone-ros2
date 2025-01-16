#include "motor_interface.h"
#include "constants.h"


double maxSpeedRPM = 1342; // Max speed in RPM is set in Clearpath Motor Setup Program (https://teknic.com/downloads/ Clearpath -> MC -> Nema 23/24 -> Software -> motor_setup.zip)
#define INPUT_A_FILTER 20
#define GEARBOX_REDUCTION 10.71
#define RPM_PER_RADPERS 9.549297

void initialize_motors(){
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_A_DIRECT_B_PWM);

  ConnectorM0.HlfbMode(MotorDriver::HLFB_MODE_HAS_PWM);
  ConnectorM1.HlfbMode(MotorDriver::HLFB_MODE_HAS_PWM);
  ConnectorM2.HlfbMode(MotorDriver::HLFB_MODE_HAS_PWM);
  ConnectorM3.HlfbMode(MotorDriver::HLFB_MODE_HAS_PWM);

  ConnectorM0.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
  ConnectorM1.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
  ConnectorM2.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
  ConnectorM3.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

  ConnectorM0.EnableRequest(true);
  ConnectorM1.EnableRequest(true);
  ConnectorM2.EnableRequest(true);
  ConnectorM3.EnableRequest(true);
}

bool CommandVelocity(MotorDriver motor, double cmdWheelRadPerS) {
    double cmdWheelRPM = cmdWheelRadPerS * RPM_PER_RADPERS;
    double commandedRPM = cmdWheelRPM * GEARBOX_REDUCTION;

    commandedRPM = cmdWheelRadPerS;

    if (commandedRPM > maxSpeedRPM) {
      commandedRPM = maxSpeedRPM * 0.95;
    }
    else if(commandedRPM < -maxSpeedRPM){
      commandedRPM = -maxSpeedRPM * 0.95;
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
    uint8_t dutyRequest = 1.0 * abs(commandedRPM) / maxSpeedRPM * 255;
 
    // Command the move.
    motor.MotorInBDuty(dutyRequest);
 
    return true;
}


double get_wheel_abs_radpers(MotorDriver motor, bool is_fl_motor=false)
{
  MotorDriver::HlfbStates hlfbState = motor.HlfbState();

  double hlfbPercent = -1;

  if (hlfbState == MotorDriver::HLFB_HAS_MEASUREMENT) {
      hlfbPercent = motor.HlfbPercent();

      if(is_fl_motor)
      {
        hlfbPercent -= 1.48;
      }
      else {
        hlfbPercent -= 1.3; // Percent speed has a roughly 1.3 percent offset. TODO - add calibration to get rid of this.
      }
      

      if(hlfbPercent < 0)
      {
        hlfbPercent = 0;
      }
  }

  double motor_abs_rpm = hlfbPercent * maxSpeedRPM;
  double wheel_abs_rpm = motor_abs_rpm / GEARBOX_REDUCTION;
  double wheel_abs_radpers = wheel_abs_rpm / RPM_PER_RADPERS;

  

  return hlfbPercent;
}