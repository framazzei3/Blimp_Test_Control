void applyManualControl() {
  float surge = manualControlValues[0] / 100.0f;
  float yaw   = manualControlValues[1] / 100.0f;
  float heave = manualControlValues[2] / 100.0f;

  // Distribuzione corretta di surge e yaw
  leftThrust  = 0.5f * surge - 0.5f * yaw;
  rightThrust = 0.5f * surge + 0.5f * yaw;
  verticalThrust = heave;

  // Per sicurezza, ma dovrebbe già essere entro [-1, 1]
  leftThrust  = constrain(leftThrust,  -1.0f, 1.0f);
  rightThrust = constrain(rightThrust, -1.0f, 1.0f);
  verticalThrust = constrain(verticalThrust, -1.0f, 1.0f);

  setRightMotorSpeed(rightThrust);
  setLeftMotorSpeed(leftThrust);
  setVerticalMotorSpeed(verticalThrust);

  // Serial.print("Left: ");  Serial.print(leftThrust);
  // Serial.print(" Right: "); Serial.print(rightThrust);
  // Serial.print(" Heave: "); Serial.println(heave);
}

// Simplified motor control functions
void setVerticalMotorSpeed(float speed) {
  setMotorPairPWM(VERT_AIN1_PWM, VERT_AIN2_PWM, 
                  VERT_BIN1_PWM, VERT_BIN2_PWM, 
                  speed);
}

void setLeftMotorSpeed(float speed) {
  setMotorPairPWM(LEFT_AIN1_PWM, LEFT_AIN2_PWM, 
                  LEFT_BIN1_PWM, LEFT_BIN2_PWM, 
                  speed);
}

void setRightMotorSpeed(float speed) {
  setMotorPairPWM(RIGHT_AIN1_PWM, RIGHT_AIN2_PWM, 
                  RIGHT_BIN1_PWM, RIGHT_BIN2_PWM, 
                  speed);
}

/// Control a motor pair (A and B) with the same speed
void setMotorPairPWM(uint8_t pwm1A, uint8_t pwm1B,
                     uint8_t pwm2A, uint8_t pwm2B,
                     float speed) {
  speed = constrain(speed, -1.0f, 1.0f);

  int pwmValue = abs(speed) * PWM_MAX;

  // if (abs(speed) < 0.05f) {
  //   pwm.setPWM(pwm1A, 0, 0);
  //   pwm.setPWM(pwm1B, 0, 0);
  //   pwm.setPWM(pwm2A, 0, 0);
  //   pwm.setPWM(pwm2B, 0, 0);
  //   return;
  // }

  if (speed > 0) {
    pwm.setPWM(pwm1A, 0, pwmValue);
    pwm.setPWM(pwm1B, 0, 0);
    pwm.setPWM(pwm2A, 0, pwmValue);
    pwm.setPWM(pwm2B, 0, 0);
  } else {
    pwm.setPWM(pwm1A, 0, 0);
    pwm.setPWM(pwm1B, 0, pwmValue);
    pwm.setPWM(pwm2A, 0, 0);
    pwm.setPWM(pwm2B, 0, pwmValue);
  }
}

