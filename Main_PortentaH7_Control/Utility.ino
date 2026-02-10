void handleControlMode() {
  static bool modeChangeLogged = false;
  static ControlMode lastMode = MODE_SAFETY; // Track previous mode

  // Detect mode change
  if (currentControlMode != lastMode) {
    modeChangeLogged = false; // Reset on ANY mode change
    lastMode = currentControlMode;
  }

  switch (currentControlMode) {
    case MODE_SAFETY:
      if (!modeChangeLogged) {
        log("[MODE] Entered SAFETY mode");
        modeChangeLogged = true;
        stopAllMotors();
      }
      break;
      
    case MODE_MANUAL:
      if (!modeChangeLogged) {
        log("[MODE] Entered MANUAL mode");
        modeChangeLogged = true;
      }
      applyManualControl();
      
      if (millis() - lastJoystickPacket > 5000) { // 5 sec timeout
        currentControlMode = MODE_SAFETY; // Will auto-reset flag next loop
        log("[INFO] No joystick for 5 sec, enter safety mode");
      }
      break;
      
    case MODE_ALTITUDE_HOLD:
      if (!modeChangeLogged) {
        log("[MODE] Entered ALTITUDE HOLD mode with target: " + String(z_ref) + "m");
        setupVerticalControl(); // Called ONLY ONCE when entering mode
        modeChangeLogged = true;
      }
      applyVerticalControl();
      break;

    case MODE_HEADING_HOLD:
    if (!modeChangeLogged) {
        log("[MODE] Entered HEADING HOLD mode with target: " + String(yaw_ref) + "m");
        modeChangeLogged = true;
      }
      applyHeadingPID();
      break;

    case MODE_HOVER:
      if (!modeChangeLogged) {
        log("[INFO] Entered HOVER mode at setpoint: X=" + String(hoverX, 2) + 
            "m, Y=" + String(hoverY, 2) + 
            "m, Z=" + String(hoverZ, 2) + 
            "m, Yaw=" + String(hoverYaw, 1) + "°");
        modeChangeLogged = true;
      }

      applyHoverControl(); 
      break;

    case MODE_WAYPOINT:
    if (!modeChangeLogged) {
        log("[MODE] Entered WAYPOINT mode");
        modeChangeLogged = true;
      }
      applyWaypointControl();
      break;

  default:
    if (!modeChangeLogged) {
      log("[MODE] Unknown mode detected");
      modeChangeLogged = true;
    }
    currentControlMode = MODE_SAFETY; // Triggers re-log of SAFETY next loop
    break;
  }
}

// Funzione dedicata per leggere la batteria
void readBattery() {
  int rawValue = analogRead(voltagePin);
  float sensorVoltage = (rawValue * adcReference) / adcMax;
  batteryStatus.voltage = sensorVoltage * scaleFactor;
  batteryStatus.percent = getBatteryPercentage(batteryStatus.voltage);
}

// Percentuale batteria
float getBatteryPercentage(float voltage) {
  const float V_min = 6.6;  // 0% (scarica)
  const float V_max = 8.4;  // 100% (carica)

  if (voltage < V_min) voltage = V_min;
  if (voltage > V_max) voltage = V_max;

  return (voltage - V_min) / (V_max - V_min) * 100.0;
}

// // Safety check loop
// void safetyCheckLoop() {
//   // 1. WiFi connection check
//   static bool wifiAlertSent = false;
//   static bool lowBatteryAlertSent = false;  
  
//   if (WiFi.status() != WL_CONNECTED) {
//     if (!wifiAlertSent) {
//       log("[SAFETY] WiFi disconnected!");
//       wifiAlertSent = true;
//     }
//     stopAllMotors();
//     return;
//   } 
//   else if (wifiAlertSent) {
//     log("[SAFETY] WiFi reconnected");
//     wifiAlertSent = false;
//   }
// }

// ========== PERIODIC TASKS ==========
void handlePeriodicTasks() {
  static unsigned long lastStatusUpdate = 0;
  unsigned long now = millis();

  // ---- System status update every 2 second ----
  if (now - lastStatusUpdate >= 100) {
    lastStatusUpdate = now;
    String status = "[STATUS] " + getSystemStatus();
    log(status);
  }
}

String getSystemStatus() {
  String status;
  status += "Mode: " + getModeName(currentControlMode) + "\n";
  // status += "IP: " + WiFi.localIP().toString() + "\n";
  
  if (ntpInitialized) {
    status += "Time: " + getCurrentTimeString() + "\n";  
  } else {
    status += "Time: NTP not initialized\n";
  } 

  if (tofAvailable) {
    status += "ToF Altitude: ";
    status += String(tofAltitude_m, 3);
    status += " m\n";
  } else {
    status += "ToF Altitude: NOT AVAILABLE\n";
  }
  return status;
}

String getModeName(ControlMode mode) {
    switch(mode) {
        case MODE_SAFETY:        return "SAFETY";
        case MODE_MANUAL:        return "MANUAL";
        case MODE_ALTITUDE_HOLD: return "ALTITUDE_HOLD";
        case MODE_HEADING_HOLD:  return "HEADING_HOLD";
        case MODE_HOVER:         return "HOVER";
        default:                 return "UNKNOWN";
    }
}

// DEBUG PRINT
void printToF() {
  if (tofAvailable) {
    Serial.print("Altitude: ");
    Serial.print(tofAltitude_m, 3);
    Serial.print(" m");
    Serial.println();
  } else {
    Serial.println("Misura ToF non valida.");
  }
}

// Funzione per stampare i dati mocap
void printMocapData() {
  if (!mocapData.mocapValid) return;

  Serial.println("=== Motion Capture Data ===");
  Serial.print("Position (X,Y,Z): ");
  Serial.print(mocapData.posX, 3); Serial.print(", ");
  Serial.print(mocapData.posY, 3); Serial.print(", ");
  Serial.println(mocapData.posZ, 3);
  
  Serial.print("Orientation (Roll,Pitch,Yaw): ");
  Serial.print(mocapData.roll, 3); Serial.print(", ");
  Serial.print(mocapData.pitch, 3); Serial.print(", ");
  Serial.println(mocapData.yaw, 3);
  Serial.println("==========================");
}





