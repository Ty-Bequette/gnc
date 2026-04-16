// Actuators -- Thruster and pyro control
#include "Arduino.h"
#include "Actuators.h"

static bool positiveThrusterON = false;
static bool negativeThrusterON = false;
static unsigned long thrusterOnTime = 0;

void initActuators() {
  // Configure thruster pins
  pinMode(THRUSTER_POSITIVE_2, OUTPUT);
  pinMode(THRUSTER_POSITIVE_4, OUTPUT);
  pinMode(THRUSTER_NEGATIVE_1, OUTPUT);
  pinMode(THRUSTER_NEGATIVE_3, OUTPUT);

  // Ensure all outputs are off
  disableAllActuators();
  
  Serial.println("Thrusters initialized");
}

void controlThrusters(float pidOutput) {
  // Apply deadzone
  // If output is within deadzone, turn off thrusters
  if (abs(pidOutput) < PID_DEADZONE) {
    digitalWrite(THRUSTER_POSITIVE_2, LOW);
    digitalWrite(THRUSTER_POSITIVE_4, LOW);
    digitalWrite(THRUSTER_NEGATIVE_1, LOW);
    digitalWrite(THRUSTER_NEGATIVE_3, LOW);
    positiveThrusterON = false;
    negativeThrusterON = false;
    return;
  }
  
  // Determine which thruster to fire
  if (pidOutput > 0) {
    // Fire LEFT thruster (positive torque)
    if (!negativeThrusterON) {
      digitalWrite(THRUSTER_POSITIVE_2, HIGH);
      digitalWrite(THRUSTER_POSITIVE_4, HIGH);
      digitalWrite(THRUSTER_NEGATIVE_1, LOW);
      digitalWrite(THRUSTER_NEGATIVE_3, LOW);
      positiveThrusterON = true;
      negativeThrusterON = false;
      thrusterOnTime = millis();
    }
  } else {
    // Fire RIGHT thruster (negative torque)
    if (!negativeThrusterON) {
      digitalWrite(THRUSTER_POSITIVE_2, LOW);
      digitalWrite(THRUSTER_POSITIVE_4, LOW);
      digitalWrite(THRUSTER_NEGATIVE_1, HIGH);
      digitalWrite(THRUSTER_NEGATIVE_3, HIGH);
      positiveThrusterON = false;
      negativeThrusterON = true;
      thrusterOnTime = millis();
    }
  }
  
  // Enforce minimum pulse duration
  if ((positiveThrusterON || negativeThrusterON) && 
      (millis() - thrusterOnTime < MIN_THRUSTER_PULSE)) {
    // Keep thruster on for minimum duration
    return;
  }
}

void disableAllActuators() {
  digitalWrite(THRUSTER_POSITIVE_2, LOW);
  digitalWrite(THRUSTER_POSITIVE_4, LOW);
  digitalWrite(THRUSTER_NEGATIVE_1, LOW);
  digitalWrite(THRUSTER_NEGATIVE_3, LOW);
  
  positiveThrusterON = false;
  negativeThrusterON = false;
}

bool getLeftThrusterState() {
  return positiveThrusterON;
}

bool getRightThrusterState() {
  return negativeThrusterON;
}