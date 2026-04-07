#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

// #include <Arduino.h>
#include "Config.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

class PIDController {
public:
  PIDController(float kp, float kd, float ki);
  
  // Set PID gains
  void setGains(float kp, float kd, float ki);
  
  // Set setpoint (desired heading)
  void setSetpoint(float setpoint);
  
  // Compute PID output
  float compute(float currentValue, float deltaTime);
  
  // Reset PID state
  void reset();
  
  // Get last error and derivative 
  float getLastError() { return lastError; }
  float getLastDerivative() { return lastDerivative; }
  float getLastOutput() { return lastOutput; }
  
private:
  float kp, kd, ki;
  float setpoint;
  float lastError;
  float lastDerivative;
  float lastOutput;
  float integral;
  
  // Wrap angle error to -180 to +180
  float wrapAngle(float angle);
};
#endif // PID_CONTROLLER_H

// IMU (Inertial Measurement Unit) data pulling function
void IMUdata (float &zAxis)
{
  sensors_event_t event;
  bno.getEvent(&event);

  zAxis = event.orientation.z;
}

// PID loop function (from Hapsis)
PIDController::PIDController(float kp, float kd, float ki) 
  : kp(kp), kd(kd), ki(ki), setpoint(0.0), 
    lastError(0.0), lastDerivative(0.0), lastOutput(0.0), integral(0.0) {
}

void PIDController::setGains(float kp, float kd, float ki) {
  this->kp = kp;
  this->kd = kd;
  this->ki = ki;
}

void PIDController::setSetpoint(float setpoint) {
  this->setpoint = setpoint;
}

float PIDController::wrapAngle(float angle) {
  // Wrap angle to -180 to +180 range
  while (angle > 180.0) {
    angle -= 360.0;
  } 
  while (angle < -180.0) {
    angle += 360.0;
  } 
  return angle;
}

float PIDController::compute(float currentValue, float deltaTime) {
  // Calculate error with wraparound handling
  float error = wrapAngle(setpoint - currentValue);
  
  // Calculate derivative
  float derivative = 0.0;
  if (deltaTime > 0.0) {
    derivative = (error - lastError) / deltaTime;
  }
  
  // Calculate integral 
  integral += error * deltaTime;
  // Anti-windup (double check this)
  integral = constrain(integral, -100.0, 100.0);  // Anti-windup
  
  // Calculate output
  float output = (kp * error) + (kd * derivative) + (ki * integral);
  
  // Store for next iteration and logging
  lastError = error;
  lastDerivative = derivative;
  lastOutput = output;
  
  return output;
}

void PIDController::reset() {
  lastError = 0.0;
  lastDerivative = 0.0;
  lastOutput = 0.0;
  integral = 0.0;
}