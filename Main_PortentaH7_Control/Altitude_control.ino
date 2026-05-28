//==================================================
//                 VELOCITY ESTIMATION PARAMETERS 
//==================================================
float z_prev = 0.0f;
float v_raw  = 0.0f;
float v_hat_butter  = 0.0f;
float z_meas = 0.0f;

//======== Time ============
unsigned long lastExecVert = 0;
float dt_vert;
//==========================

// Filtered derivative time constant (LOWPASS FILTER)
const float T_derivative = 0.2f;  // [s] - Fast response for derivative
float alpha_d = 0.0f;              // Updated in loop based on dt_vert

//==================================================
//                 GAIN SCHEDULING PARAMETERS                    
//==================================================
float v_th = 0.05f;
// bool is_up = true;

//==================================================
//              VELOCITY PI CONTROLLER               
//==================================================
// PID variables (outer loop)
float v_ref        = 0.0f;
float v_ref_unsat  = 0.0f;
float e_v          = 0.0f;  // Error
float int_ev       = 0.0f;  // Integral error
float de_v_dt      = 0.0f;  // Derivative error
float de_v_dt_filt = 0.0f;  // Derivative filtered error
float e_v_prev     = 0.0f;

// Gains
float Kp_down = 3.1f;
float Kd_down = 0.08f;   
float Ki_down = 1.5f;

// float Kp_up   = 2.5f;
// float Kd_up   = 2.0f;
// float Ki_up   = 0.4f;

float Kp_up   = 6.0f;
float Kd_up   = 0.0f;
float Ki_up   = 2.0f;

float Kp_v = Kp_up;
float Ki_v = Ki_up;
float Kd_v = Kd_up;

//==================================================
//               ALTITUDE CONTROL                   
//==================================================
float z_ref  = 1.0f;   // target altitude [m]
// float Kp_z   = 0.5f;   // altitude -> velocity gain
float Kp_z   = 0.35f;   // altitude -> velocity gain
float uz_pid = 0.0f;   // Total PID command

//==================================================
//               BUTTERWORTH FILER                 
//==================================================
// Sampling frequency and cutoff
const double f_s = 20.0;   // your 20 Hz control loop
const double f_c = 1;    // cutoff frequency ~ 5 Hz
const double f_n = 2 * f_c / f_s;

// Butterworth filter instance (2nd order)
auto vz_filter = butter<2>(f_n);
//==================================================
//                  SETUP                         
//==================================================

void setupVerticalControl() {
  log("[INIT] Vertical control initialized");

  lastExecVert = 0;
  int_ev       = 0.0f;
  e_v_prev     = 0.0f;
  de_v_dt_filt = 0.0f;

  vz_filter = butter<2>(f_n); 
  v_hat_butter = 0.0f;
  v_raw = 0.0f;
  
  // Initialize z_prev with current measurement
  // z_meas = tofAltitude_m;
  z_meas = mocapData.posZ;
  z_prev = z_meas;

  // Crea il file CSV se il logging è attivo
  if (sdLoggingEnabled && sdCardInitialized) {
    if (currentControlMode == MODE_ALTITUDE_HOLD) {
      String folderPath = "/fs/" + getDateFolder();
      filename = folderPath + "/" + getDateTimeFilename();
      
      FILE* file = fopen(filename.c_str(), "w");
      if (file) {
          writeCSVHeaderAltitude(file);
          fclose(file);
          log("[SD] Log file created: " + filename);
      }
    }
  }
}

//==================================================
//                MAIN CONTROL LOOP                 
//==================================================

void applyVerticalControl() {
  // Safety
  // if (!mocapData.mocapValid) {
  //   log("[WARN] MoCap invalid - switching to SAFETY");
  //   currentControlMode = MODE_SAFETY;
  //   return;
  // }

  // Timing (50 ms)
  unsigned long now = millis();

  // First execution: initialize only
  if (lastExecVert == 0) {
    lastExecVert = now;
    return;
  }

  if (now - lastExecVert < 50) return; // 50 ms = 20 Hz
  
  dt_vert = (now - lastExecVert) * 1e-3f;
  lastExecVert = now;


  // ======================
  // VELOCITY ESTIMATION (MOVING AVERAGE ON v_raw)
  // ======================

  // --- Acquire altitude from ToF sensor ---
  z_meas = mocapData.posZ;
  // z_meas = tofAltitude_m;

  // Raw derivative
  v_raw = (z_meas - z_prev) / dt_vert;

  // Applica il filtro Butterworth alla derivata
  v_hat_butter = vz_filter(v_raw);

  // ======================
  // Update z_prev for next iteration
  // ======================
  z_prev = z_meas;

  // ======================
  // ALTITUDE -> VELOCITY (OUTER LOOP)
  // ======================
  v_ref_unsat = Kp_z * (z_ref - z_meas); 
  v_ref = constrain(v_ref_unsat, -1.0f, 1.0f);

  // ======================
  // GAIN SCHEDULING (Optional - currently using up gains only)
  // ======================
  // Uncomment below for gain scheduling based on direction
  /*
  if (is_up && v_ref < -v_th) {
      is_up = false;
  }
  else if (!is_up && v_ref > v_th) {
      is_up = true;
  }

  float Kp_v = is_up ? Kp_up : Kp_down;
  float Ki_v = is_up ? Ki_up : Ki_down;
  float Kd_v = is_up ? Kd_up : Kd_down;
  */
  
  // ======================
  // VELOCITY CONTROL (INNER LOOP)
  // ======================
  // Update alpha_d based on dt and T_derivative
  alpha_d = dt_vert / (T_derivative + dt_vert);  

  e_v = v_ref - v_hat_butter;                // Current error
  int_ev += e_v * dt_vert;                   // Error integral
  de_v_dt = (e_v - e_v_prev) / dt_vert;      // Error derivative
  
  // Low-pass filter on derivative (exponential smoothing)
  de_v_dt_filt = alpha_d * de_v_dt + (1.0f - alpha_d) * de_v_dt_filt;
  
  e_v_prev = e_v;  // Update error for next loop

  // PID control law
  uz_pid = Kp_v * e_v + Ki_v * int_ev + Kd_v * de_v_dt_filt;

  // Saturation + anti-windup
  if (uz_pid > 1.0f) {                    
    uz_pid = 1.0f;                              // Clamp to maximum
    int_ev -= e_v * dt_vert;                    // Undo last integral (anti-windup)
  } else if (uz_pid < -1.0f) {       
    uz_pid = -1.0f;                              // Clamp to minimum
    int_ev -= e_v * dt_vert;                    // Undo last integral (anti-windup)
  }

  // Send command to vertical motor
  setVerticalMotorSpeed(uz_pid);
  
    // Log solo se siamo in ALTITUDE_HOLD, non in HEADING_HOLD
    if (currentControlMode == MODE_ALTITUDE_HOLD) {
        logCSVAltitudeRow();
    }

}

void logVerticalControl() {
  if (currentControlMode != MODE_ALTITUDE_HOLD) return;

  String msg = "[VERT] ";

  msg += "z="     + String(z_meas, 3) + ", ";
  msg += "v_raw=" + String(v_raw, 3)  + ", ";
  msg += "v_hat=" + String(v_hat_butter, 3)  + ", ";
  msg += "v_ref=" + String(v_ref, 3)  + ", ";
  msg += "e_v="   + String(e_v, 3)    + ", ";
  msg += "int="   + String(int_ev, 3) + ", ";
  msg += "u="     + String(uz_pid, 3);

  log(msg);
}

void logCSVAltitudeRow() {
    if (!sdLoggingEnabled || !sdCardInitialized) return;
    if (filename == "") return;
    if (currentControlMode == MODE_WAYPOINT) return;  


    FILE* file = fopen(filename.c_str(), "a");
    if (!file) return;

    fprintf(file, "%lu,%s,%s,",
            millis(),
            getCurrentTimeString().c_str(),
            getModeName(currentControlMode).c_str());

    fprintf(file, "%.2f,%.1f,",     batteryStatus.voltage, batteryStatus.percent);
    fprintf(file, "%.4f,%.4f,",     dt_vert, alpha_d);
    fprintf(file, "%.4f,%.4f,%.4f,", z_ref, z_meas, tofAltitude_m);
    fprintf(file, "%.4f,%.4f,%d,",  mocapData.posX, mocapData.posY, (int)mocapData.mocapValid);
    fprintf(file, "%.4f,%.4f,",     v_raw, v_hat_butter);
    fprintf(file, "%.4f,%.4f,",     v_ref_unsat, v_ref);
    fprintf(file, "%.4f,%.4f,%.4f,%.4f,", e_v, int_ev, de_v_dt, de_v_dt_filt);
    fprintf(file, "%.4f,",          uz_pid);
    fprintf(file, "%.3f,%.3f,%.3f,%.3f,", Kp_v, Ki_v, Kd_v, Kp_z);

    fclose(file);
} 


void writeCSVHeaderAltitude(FILE* file) {
    fprintf(file,
        "millis,time,mode,"
        "bat_voltage,bat_percent,"
        "dt_vert,alpha_d,"
        "z_ref,z_meas,tof_alt,"
        "mocap_x,mocap_y,mocap_valid,"
        "v_raw,v_hat_butter,"
        "v_ref_unsat,v_ref,"
        "e_v,int_ev,de_v_dt,de_v_dt_filt,"
        "uz_pid,"
        "Kp_v,Ki_v,Kd_v,Kp_z,"
    );
}

