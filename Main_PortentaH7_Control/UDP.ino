void log(String message) {
  if (Udp.beginPacket(remoteIP, remotePort)) {
      Udp.write(message.c_str());
      Udp.endPacket();
    // Optional: Print to serial (local debug)
  Serial.println(message);  
  }
}

void handleUDPCommunication() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = '\0';
      String command = String(packetBuffer);
      command.replace("\r", "");
      command.replace("\n", "");
      command.trim();
      
      // Debug print
      // Serial.print("Command raw: [");
      // Serial.print(command);
      // Serial.println("]");
      
      if (command.equals("?")) {
        getSystemStatus();
      }

      // ===================== SD LOG CONTROL ======================
      else if (command.equalsIgnoreCase("log:on")) {
        sdLoggingEnabled = true;
        filename = "";                // <--- forza creazione nuovo file
        log("[SD] Logging ENABLED");
      }

      else if (command.equalsIgnoreCase("log:off")) {
        sdLoggingEnabled = false;
        log("[SD] Logging DISABLED");
      }

      else if (command.equals("s")) {
        currentControlMode = MODE_SAFETY;
        // log("[INFO] Command: Enter safety mode");
      }
      else if (command.equals("m")) {
        currentControlMode = MODE_MANUAL;
        lastJoystickPacket = millis();
        // log("[INFO] Command: Enter manual mode");
      }
      else if (currentControlMode == MODE_MANUAL && command.startsWith("S:")) {
        parseJoystickPacket(command.c_str());
      }
      else if (command.startsWith("z:")) {  
          String altStr = command.substring(2);
          float newAlt = altStr.toFloat();
          
          z_ref = newAlt;
          currentControlMode = MODE_ALTITUDE_HOLD;
          // log("[INFO] Command: Enter altitude hold mode with target: " + String(targetAltitude, 2) + "m");
      }

      // Formato atteso: GAINS:Kp_up,Kd_up,Ki_up,Kp_z
      else if (command.startsWith("gains:")) {

        String values = command.substring(6);  
        int idx1 = values.indexOf(',');
        int idx2 = values.indexOf(',', idx1 + 1);
        int idx3 = values.indexOf(',', idx2 + 1);

        if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {

            Kp_up = values.substring(0, idx1).toFloat();
            Kd_up = values.substring(idx1 + 1, idx2).toFloat();
            Ki_up = values.substring(idx2 + 1, idx3).toFloat();
            Kp_z = values.substring(idx3 + 1).toFloat();

            // Aggiorno anche i valori vincolati
            Kp_v = Kp_up;
            Ki_v = Ki_up;
            Kd_v = Kd_up;

            log("Parametri aggiornati:");
            log("Kp_up = " + String(Kp_up, 3));
            log("Kd_up = " + String(Kd_up, 3));
            log("Ki_up = " + String(Ki_up, 3));
            log("Kp_z  = " + String(Kp_z, 3));

        }
        else {
            log("Formato GAINS non valido!");
        }
      }

      else if (command.startsWith("h:")) {  // New heading command
        String yawStr = command.substring(2);
        yaw_ref = yawStr.toFloat();
        
        currentControlMode = MODE_HEADING_HOLD;
        // log("[INFO] Command: Enter heading hold mode with target: " + String(yawSetpoint, 2) + "°");
      }
      else if (command.startsWith("hover:")) {
        int sep1 = command.indexOf(':');
        String values = command.substring(sep1 + 1);

        // Trova tutte le posizioni delle virgole
        int sep[4];
        sep[0] = -1;
        for(int i = 1; i < 4; i++) {
            sep[i] = values.indexOf(',', sep[i-1] + 1);
        }

        // Verifica che tutte le virgole siano state trovate
        if (sep[1] > 0 && sep[2] > 0 && sep[3] > 0) {
            // Estrai i valori
            hoverX = values.substring(0, sep[1]).toFloat();
            hoverY = values.substring(sep[1] + 1, sep[2]).toFloat();
            hoverZ = values.substring(sep[2] + 1, sep[3]).toFloat();
            hoverYaw = values.substring(sep[3] + 1).toFloat(); //deg

            // Aggiorna i setpoint dei PID (solo quota)
            z_ref = hoverZ;

            currentControlMode = MODE_HOVER;
        } else {
            log("[ERROR] Invalid HOVER command format. Expected: HOVER:X,Y,Z,YAW");
            log("[DEBUG] Received: " + command);
        }
      }
      else if (command.startsWith("mission:")) {
        String data = command.substring(8);  // Rimuove "mission:"
        waypointCount = 0;                   // Reset coda

        int start = 0;
        int sepIndex;
        
        // Divide in waypoint separati da ';'
        while (waypointCount < MAX_WAYPOINTS && (sepIndex = data.indexOf(';', start)) != -1) {
          String wpStr = data.substring(start, sepIndex);
          start = sepIndex + 1;

          int c1 = wpStr.indexOf(',');
          int c2 = wpStr.indexOf(',', c1 + 1);
          int c3 = wpStr.indexOf(',', c2 + 1);

          if (c1 > 0 && c2 > 0 && c3 > 0) {
            float x = wpStr.substring(0, c1).toFloat();
            float y = wpStr.substring(c1 + 1, c2).toFloat();
            float z = wpStr.substring(c2 + 1, c3).toFloat();
            float yaw = wpStr.substring(c3 + 1).toFloat(); // deg

            waypointQueue[waypointCount++] = {x, y, z, yaw};
            log("[MISSION] Added WP#" + String(waypointCount-1) + 
                          " -> X=" + String(x,2) + 
                          " Y=" + String(y,2) + 
                          " Z=" + String(z,2) + 
                          " Yaw=" + String(yaw,1) + "°");
          }
        }

        // Ultimo waypoint (dopo l’ultimo ';')
        if (waypointCount < MAX_WAYPOINTS && start < data.length()) {
          String wpStr = data.substring(start);
          int c1 = wpStr.indexOf(',');
          int c2 = wpStr.indexOf(',', c1 + 1);
          int c3 = wpStr.indexOf(',', c2 + 1);

          if (c1 > 0 && c2 > 0 && c3 > 0) {
            float x = wpStr.substring(0, c1).toFloat();
            float y = wpStr.substring(c1 + 1, c2).toFloat();
            float z = wpStr.substring(c2 + 1, c3).toFloat();
            float yaw = wpStr.substring(c3 + 1).toFloat(); // deg

            waypointQueue[waypointCount++] = {x, y, z, yaw};
            log("[MISSION] Added WP#" + String(waypointCount-1) + 
                          " -> X=" + String(x,2) + 
                          " Y=" + String(y,2) + 
                          " Z=" + String(z,2) + 
                          " Yaw=" + String(yaw,1) + "°");
          }
        }
        

          if (waypointCount > 0) {
            currentWaypointIndex = 0;
            waypointReached = false;
            missionActive = true;
            currentControlMode = MODE_WAYPOINT;
            log("[MISSION] Loaded " + String(waypointCount) + " waypoints, starting mission.");
          } else {
            log("[ERROR] No valid waypoint parsed from: " + command);
        }
      }
      else if (command.startsWith("qtm:")) {
        String data = command.substring(4); // Rimuove "qtm:"
        float values[6];
        int idx = 0;
        int start = 0;
        int commaIndex;

        // Parse comma-separated values
        while (idx < 6 && (commaIndex = data.indexOf(',', start)) != -1) {
          values[idx++] = data.substring(start, commaIndex).toFloat();
          start = commaIndex + 1;
        }
        // Get last value after the last comma
        if (idx < 6) {
          values[idx++] = data.substring(start).toFloat();
        }

        if (idx == 6) {
          // Controllo NaN
          bool anyNaN = false;
          for (int i = 0; i < 6; i++) {
            if (isnan(values[i])) {
              anyNaN = true;
              break;
            }
          }
          if (anyNaN) {
            mocapData.mocapValid = false;
            // Reset a NaN quando ricevi dati corrotti
            mocapData.posX = NAN;
            mocapData.posY = NAN;
            mocapData.posZ = NAN;
            mocapData.roll = NAN;
            mocapData.pitch = NAN;
            mocapData.yaw = NAN;
            mocapData.mocapValid = false;                
          } else {
            // Update motion capture data
            mocapData.posX = values[0];
            mocapData.posY = values[1];
            mocapData.posZ = values[2];
            mocapData.roll = values[3];
            mocapData.pitch = values[4];
            mocapData.yaw = values[5];
            mocapData.mocapValid = true; 
          }
        } else {
            log("[ERROR] Invalid QTM packet (expected 6 values, got " + String(idx) + "): " + command);
        }
      } else {
        log("Unknown command: " + command);
      }
    }
  }
}

void parseJoystickPacket(const char* packet) {
  // Debug: Print received packet
  // Serial.print("Received: ");
  // Serial.println(packet);

  // Reset manual control values
  for (int i = 0; i < 3; i++) {
    manualControlValues[i] = 0;
  }

  // Variables to store parsed values
  int surge = 0, yaw = 0, heave = 0;

  // Ensure packet format is correct before parsing
  if (sscanf(packet, "S:%d,Y:%d,H:%d", &surge, &yaw, &heave) == 3) {
    // Assign values to manual control array
    manualControlValues[0] = surge;
    manualControlValues[1] = yaw;
    manualControlValues[2] = heave;

    // Debug: Print parsed values
    // Serial.print("Parsed -> Surge: ");
    // Serial.print(surge);
    // Serial.print(" | Yaw: ");
    // Serial.print(yaw);
    // Serial.print(" | Heave: ");
    // Serial.println(heave);

    // Update timestamp
    lastJoystickPacket = millis();
  } else {
    log("Error: Failed to parse joystick packet. Check format.");
  }
}

// char udpBuffer[256];   // Assicurati che sia abbastanza grande

// // Costruisci la stringa CSV esattamente come su SD
// snprintf(udpBuffer, sizeof(udpBuffer),
//         "%lu,%s,%s,"         // timestamp, time string, mode name
//         "%.2f,%.1f,"         // battery
//         "%.4f,"              // dt
//         "%.4f,%.4f,%.4f,"    // velocities
//         "%.4f,%.4f,"         // altitude
//         "%.4f,%.4f,%.4f,%.4f,%.4f,"  // PID
//         "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f," // gains
//         "%.4f,"              // ToF
//         "%.4f,%.4f,%.4f",    // thrusts
//         currentTime, 
//         getCurrentTimeString().c_str(), 
//         getModeName(currentControlMode).c_str(),
//         batteryStatus.voltage,
//         batteryStatus.percent,
//         dt_vert,
//         v_raw, v_hat_butter, v_ref,
//         z_ref, z_meas,
//         e_v, int_ev, de_v_dt, de_v_dt_filt, uz_pid,
//         Kp_up, Ki_up, Kd_up, Kp_down, Ki_down, Kd_down, Kp_z,
//         tofAltitude_m,
//         leftThrust, rightThrust, verticalThrust
// );

// // Invia via UDP
// log(String(udpBuffer));




