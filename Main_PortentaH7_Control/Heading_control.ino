//==================================================
//               HEADING CONTROL                   
//==================================================
float yaw_ref      = 0.0;
const float Kp_psi = 1.2;    // guadagno angolo → yaw rate
float u_yaw_pid    = 0.0f;   // Total PID command
float w_raw        = 0.0f;   

//==================================================
//                 VELOCITY ESTIMATION PARAMETERS 
//==================================================
float dt_head      = 0.0f;   // FIX: inizializzazione
const float alpha_w = 0.85;   // filtro esponenziale low-pass

float w_hat       = 0.0;      // yaw rate filtrato
float w_prev      = 0.0;
float e_psi       = 0.0f;     
float e_w         = 0.0f;     

//==================================================
//              VELOCITY PI CONTROLLER               
//==================================================
const float Kp_w = 0.8;
const float Ki_w = 0.3;

// ========== PARAMETRI CONTROLLO ANGOLARE (outer loop) ==========
const float W_MAX  = 1.5;     // saturazione yaw-rate reference (rad/s)

float yawInput    = 0.0;

// Heading control
unsigned long lastExecYaw = 0; 
float int_yaw = 0.0f;


void setupHeadingPID() {
  // Inizializzazione se necessaria
  int_yaw = 0.0f;
  w_hat = 0.0f;
  w_prev = 0.0f;
}

void applyHeadingPID() {

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
    w_prev = mocapData.yaw;  // FIX: inizializza w_prev al primo ciclo
    return;
  }

 if (now - lastExecYaw < 100) return; // 100 ms = 10 Hz

  dt_head = (now - lastExecYaw) * 1e-3f;  // seconds
  lastExecYaw = now;

  // ---- 2. Lettura yaw ----
  yawInput = mocapData.yaw; // (deg)

  // ---- 3. Stima yaw rate w_hat ----
  w_raw = (yawInput - w_prev) / dt_head;
  w_prev = yawInput;

  // filtro esponenziale
  w_hat = alpha_w * w_hat + (1.0f - alpha_w) * w_raw;

  // ---- 4. Outer loop: angle → yaw rate reference ----
  e_psi = yaw_ref - yawInput;

  // wrapping in [-180, 180]
  while (e_psi > 180.0f)  e_psi -= 360.0f;
  while (e_psi < -180.0f) e_psi += 360.0f;

  float w_ref = Kp_psi * e_psi;

  // saturazione
  if (w_ref > W_MAX)  w_ref = W_MAX;
  else if (w_ref < -W_MAX) w_ref = -W_MAX;

  // ---- 5. Inner loop PI su yaw rate ----
  e_w = w_ref - w_hat;

  // integratore
  int_yaw += dt_head * e_w;

  // controllo totale
  u_yaw_pid = Kp_w * e_w + Ki_w * int_yaw;

  // ---- 6. Saturazione e anti-windup ----
  if (u_yaw_pid > 1.0f) {
    u_yaw_pid = 1.0f;
    int_yaw -=  dt_head * e_w;   // antiwindup
  }
  else if (u_yaw_pid < -1.0f) {
    u_yaw_pid = -1.0f;
    int_yaw -= dt_head * e_w;   // antiwindup
  }

  // ---- 7. Output ai motori ----
  setLeftMotorSpeed( 0.5f * u_yaw_pid );
  setRightMotorSpeed( -0.5f * u_yaw_pid );

}