//==================================================
//               HEADING CONTROL                   
//==================================================
float yaw_ref      = 0.0;
float u_yaw_pid    = 0.0f;   // Total PID command
float w_raw        = 0.0f;   
const float W_MAX  = 1.0f;    // rad/s — velocità angolare massima
const float COMP_FACTOR = 2.0f;  // sqrt(3.15/0.8) ≈ 2.0
// ======== Soglia fisica motori ========
const float MOTOR_THRESHOLD = 0.15f;   // sotto questa soglia il motore non gira

// ======== Comandi motori ========
float left_cmd  = 0.0f;
float right_cmd = 0.0f;
//==================================================
//                 VELOCITY ESTIMATION PARAMETERS 
//==================================================
float dt_head      = 0.0f;   // FIX: inizializzazione

// Butterworth filter instance (1nd order)
auto omegaFilter = butter<1>(f_n);

float w_hat_butter = 0.0;      // yaw rate filtrato
float w_prev       = 0.0;
float w_ref        = 0.0;
float e_psi        = 0.0f;     
float e_w          = 0.0f;     

//==================================================
//              VELOCITY PI CONTROLLER               
//==================================================
float Kp_psi = 0.5;   // angolo → yaw rate
float Kp_w   = 0.8;   // PI yaw rate
float Ki_w   = 0.3;

float yaw_meas    = 0.0;

// Heading control
unsigned long lastExecYaw = 0; 
float int_ew = 0.0f;


void setupHeadingControl() {
  // Inizializzazione se necessaria
  log("[INIT] Heading PID initialized");

  lastExecYaw = 0;
  int_ew = 0.0f;
  w_hat_butter = 0.0f;
  omegaFilter = butter<1>(f_n); 

  // yaw_meas = mocapData.yaw;
  yaw_meas = radians(mocapData.yaw);

  w_prev = yaw_meas;

  // Crea il file CSV se il logging è attivo
  if (sdLoggingEnabled && sdCardInitialized) {
    if (currentControlMode == MODE_HEADING_HOLD) {
      String folderPath = "/fs/" + getDateFolder();
      filename = folderPath + "/" + getDateTimeFilename();
      
      FILE* file = fopen(filename.c_str(), "w");
      if (file) {
          writeCSVHeaderHeading(file);
          fclose(file);
          log("[SD] Log file created: " + filename);
      }
    }
  }
}

void applyHeadingControl() {

  // Safety check
  if (!mocapData.mocapValid) {
    log("[WARN] MoCap data invalid - switching to SAFETY mode");
    currentControlMode = MODE_SAFETY;
    return;
  }

  unsigned long now = millis();

  // Prima esecuzione: inizializza e basta
  if (lastExecYaw == 0) {
    lastExecYaw = now;
    return;
  }

 if (now - lastExecYaw < 50) return; // 50 ms = 20 Hz

  dt_head = (now - lastExecYaw) * 1e-3f;  // seconds
  lastExecYaw = now;

  // ---- 2. Lettura yaw ----
  yaw_meas = radians(mocapData.yaw);    // rad
  float yaw_ref_rad = radians(yaw_ref); // rad

  // ---- 3. Stima yaw rate w_hat ----
  float dyaw = wrapToPi(yaw_meas - w_prev);
  w_raw = dyaw / dt_head;
  
  // Applica il filtro Butterworth alla derivata
  w_hat_butter = omegaFilter(w_raw);

  w_prev = yaw_meas;

  // ---- Outer loop ----
  e_psi = wrapToPi(yaw_ref_rad - yaw_meas);
  w_ref = constrain(Kp_psi * e_psi, -W_MAX, W_MAX);

  // ---- 5. Inner loop PI su yaw rate ----
  e_w = w_ref - w_hat_butter;

  // integratore
  int_ew += dt_head * e_w;

  // controllo totale
  u_yaw_pid = Kp_w * e_w + Ki_w * int_ew;

  // ---- 6. Saturazione e anti-windup ----
  if (u_yaw_pid > 1.0f) {
    u_yaw_pid = 1.0f;
    int_ew -=  dt_head * e_w;   // antiwindup
  }
  else if (u_yaw_pid < -1.0f) {
    u_yaw_pid = -1.0f;
    int_ew -= dt_head * e_w;   // antiwindup
  }

  // ---- 7. Output ai motori CON COMPENSAZIONE ----

  if (u_yaw_pid >= 0) {
    // Ruota a destra: destra avanti, sinistra indietro
    right_cmd = u_yaw_pid;                    // avanti (positivo)
    left_cmd = -u_yaw_pid * COMP_FACTOR;      // indietro compensato (negativo)
  } else {
    // Ruota a sinistra: sinistra avanti, destra indietro
    left_cmd = -u_yaw_pid;                   // avanti (positivo, perché u_yaw_pid è negativo)
    right_cmd = u_yaw_pid * COMP_FACTOR;     // indietro compensato (negativo)
  }

  // Saturazione ai limiti fisici dei motori [-1, 1]
  left_cmd = constrain(left_cmd, -1.0f, 1.0f);
  right_cmd = constrain(right_cmd, -1.0f, 1.0f);

      // Azzeramento fisico: sotto soglia manda 0 al motore
  if (fabsf(left_cmd)  < MOTOR_THRESHOLD) left_cmd  = 0.0f;
  if (fabsf(right_cmd) < MOTOR_THRESHOLD) right_cmd = 0.0f;

  // Applica ai motori
  setLeftMotorSpeed(left_cmd);
  setRightMotorSpeed(right_cmd);

  // // // ---- 7. Output ai motori ----
  // setLeftMotorSpeed( -0.5f * u_yaw_pid );
  // setRightMotorSpeed( 0.5f * u_yaw_pid );
  
  logCSVHeadingRow();
  // logYawControl();
}

void logYawControl() {
  if (currentControlMode != MODE_HEADING_HOLD) return;

  String msg = "[YAW] ";

  msg += "psi="     + String(yaw_meas, 3) + ", ";
  msg += "w_raw=" + String(w_raw, 3)  + ", ";
  msg += "w_hat=" + String(w_hat_butter, 3)  + ", ";
  msg += "w_ref=" + String(w_ref, 3)  + ", ";
  msg += "e_psi=" + String(e_psi,3) + ", ";
  msg += "e_w="   + String(e_w, 3)    + ", ";
  msg += "int="   + String(int_ew, 3) + ", ";
  msg += "u="     + String(u_yaw_pid, 3);

  log(msg);
}


void logCSVHeadingRow() {
    if (!sdLoggingEnabled || !sdCardInitialized) return;
    if (filename == "") return;

    FILE* file = fopen(filename.c_str(), "a");
    if (!file) return;

    fprintf(file, "%lu,%s,%s,",
            millis(),
            getCurrentTimeString().c_str(),
            getModeName(currentControlMode).c_str());

    fprintf(file, "%.2f,%.1f,",         batteryStatus.voltage, batteryStatus.percent);
    fprintf(file, "%.4f,",              dt_head);
    fprintf(file, "%.4f,%.4f,%.4f,",   z_ref, z_meas, tofAltitude_m);      // verticale (viene da setupVerticalControl)
    fprintf(file, "%.4f,%.4f,%.4f,%d,", mocapData.posX, mocapData.posY, mocapData.yaw, (int)mocapData.mocapValid);
    fprintf(file, "%.4f,%.4f,",         w_raw, w_hat_butter);
    fprintf(file, "%.4f,",              w_ref);
    fprintf(file, "%.4f,%.4f,%.4f,",   e_psi, e_w, int_ew);
    fprintf(file, "%.4f,",             u_yaw_pid);
    fprintf(file, "%.3f,%.3f,%.3f,",   Kp_psi, Kp_w, Ki_w);
    fprintf(file, "%.3f,%.3f,%.3f,%.3f,", Kp_v, Ki_v, Kd_v, Kp_z); 
    fprintf(file, "%.4f,%.4f,",     v_ref, uz_pid);
    fprintf(file, "%.4f,%.4f\n",    left_cmd, right_cmd);
    fclose(file);
}

void writeCSVHeaderHeading(FILE* file) {
    fprintf(file,
        "millis,time,mode,"
        "bat_voltage,bat_percent,"
        "dt_head,"
        "z_ref,z_meas,tof_alt,"
        "mocap_x,mocap_y,mocap_yaw,mocap_valid,"
        "w_raw,w_hat_butter,"
        "w_ref,"
        "e_psi,e_w,int_ew,"
        "u_yaw_pid,"
        "Kp_psi,Kp_w,Ki_w,"
        "Kp_v,Ki_v,Kd_v,Kp_z,"  
        "v_ref,uz_pid,"
        "left_cmd,right_cmd\n"
    );
}