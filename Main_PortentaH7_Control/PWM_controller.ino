void initializePWM() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(1600);  // Adatta per DRV8833

  Wire.setClock(400000);  // I2C più veloce
  
  // Inizializza solo i pin SLP
  initializeMotorGroup(VERT_SLP);
  initializeMotorGroup(LEFT_SLP);
  initializeMotorGroup(RIGHT_SLP);

  stopAllMotors();

  Serial.println("[INFO]: Motors initialized (PWM-only control)");
}

void initializeMotorGroup(uint8_t slpPin) {
  pinMode(slpPin, OUTPUT);
  digitalWrite(slpPin, HIGH);
}

void stopMotorGroup(uint8_t pwmPin1, uint8_t pwmPin2) {
  pwm.setPWM(pwmPin1, 0, 0);
  pwm.setPWM(pwmPin2, 0, 0);
}


void stopMotorGroup(uint8_t pwmPinA, uint8_t dirPinA, uint8_t pwmPinB, uint8_t dirPinB) {
  analogWrite(pwmPinA, 0);
  analogWrite(pwmPinB, 0);
  digitalWrite(dirPinA, LOW);
  digitalWrite(dirPinB, LOW);
}

void stopAllMotors() {
  stopMotorGroup(VERT_AIN1_PWM, VERT_AIN2_PWM);
  stopMotorGroup(VERT_BIN1_PWM, VERT_BIN2_PWM);
  
  stopMotorGroup(LEFT_AIN1_PWM, LEFT_AIN2_PWM);
  stopMotorGroup(LEFT_BIN1_PWM, LEFT_BIN2_PWM);
  
  stopMotorGroup(RIGHT_AIN1_PWM, RIGHT_AIN2_PWM);
  stopMotorGroup(RIGHT_BIN1_PWM, RIGHT_BIN2_PWM);

  leftThrust = 0.0f;
  rightThrust = 0.0f;
  verticalThrust = 0.0f;

}


