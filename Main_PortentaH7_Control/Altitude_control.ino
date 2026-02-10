//==================================================
//                 VELOCITY ESTIMATION PARAMETERS 
//==================================================
float z_prev = 0.0f;
float v_raw  = 0.0f;
float v_hat  = 0.0f;
float v_hat_butter  = 0.0f;
float z_meas = 0.0f;

unsigned long lastExecVert = 0;
float dt_vert;

// Filtered derivative time constant (LOWPASS FILTER)
const float T_derivative = 0.2f;  // [s] - Fast response for derivative
float alpha_d = 0.0f;              // Updated in loop based on dt_vert

//==================================================
//                 GAIN SCHEDULING PARAMETERS                    
//==================================================
float v_th = 0.05f;
bool is_up = true;

//==================================================
//              VELOCITY PI CONTROLLER               
//==================================================
// PID variables (outer loop)
float v_ref    = 0.0f;
float e_v          = 0.0f;  // Error
float int_ev       = 0.0f;  // Integral error
float de_v_dt      = 0.0f;  // Derivative error
float de_v_dt_filt = 0.0f;  // Derivative filtered error
float e_v_prev     = 0.0f;

// Gains
float Kp_up   = 2.5f;
float Kd_up   = 1.0f;
float Ki_up   = 0.1f;
float Kp_down = 3.1f;
float Kd_down = 0.08f;   
float Ki_down = 1.5f;

float Kp_v = Kp_up;
float Ki_v = Ki_up;
float Kd_v = Kd_up;

//==================================================
//               ALTITUDE CONTROL                   
//==================================================
float z_ref  = 0.5f;   // target altitude [m]
float Kp_z   = 0.5f;   // altitude -> velocity gain
float uz_pid = 0.0f;   // Total PID command

//==================================================
//               BUTTERWORTH FILER                 
//==================================================
// Sampling frequency and cutoff
const double f_s = 10.0;   // your 10 Hz control loop
const double f_c = 0.5;    // cutoff frequency ~ 0.5 Hz
const double f_n = 2 * f_c / f_s;

// Butterworth filter instance (2nd order)
auto velFilter = butter<2>(f_n);
//==================================================
//                  SETUP                         
//==================================================

void setupVerticalControl() {
  log("[INIT] Vertical control initialized");

  lastExecVert = 0;
  int_ev       = 0.0f;
  e_v_prev     = 0.0f;
  de_v_dt_filt = 0.0f;

  v_hat = 0.0f;
  velFilter = butter<2>(f_n); // ricrea oggetto -> stato viene azzerato
  v_hat_butter = 0.0f;
  v_raw = 0.0f;
  
  // Initialize z_prev with current measurement
  z_meas = tofAltitude_m;
  z_prev = z_meas;
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

  if (now - lastExecVert < 100) return; // 100 ms = 10 Hz
  
  dt_vert = (now - lastExecVert) * 1e-3f;
  lastExecVert = now;

  // --- Acquire altitude from ToF sensor ---
    // float z_meas = mocapData.posZ;
  z_meas = tofAltitude_m;

  // ======================
  // VELOCITY ESTIMATION (MOVING AVERAGE ON v_raw)
  // ======================

  // Raw derivative
  v_raw = (z_meas - z_prev) / dt_vert;

  // Applica il filtro Butterworth alla derivata
  v_hat_butter = velFilter(v_raw);

  // ======================
  // Update z_prev for next iteration
  // ======================
  z_prev = z_meas;

  // ======================
  // ALTITUDE -> VELOCITY (OUTER LOOP)
  // ======================
  v_ref = Kp_z * (z_ref - z_meas);
  v_ref = constrain(v_ref, -1.0f, 1.0f);

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


