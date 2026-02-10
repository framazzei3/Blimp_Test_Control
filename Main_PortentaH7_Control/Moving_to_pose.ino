// ======== Parametri del controllore ========
float hoverX = 0.0;   // m
float hoverY = 0.0;   // m
float hoverZ = 1.0;   // m
float hoverYaw = 0.0; // deg

// Parametri geometrici
const float L = 0.2f; // Distanza tra le ruote (m)

// ---- PI parameters ----
float Kp_v_hover = 1.2f;
float Ki_v_hover = 3.0f;

float Kp_w_hover = 2.0f;
float Ki_w_hover = 4.0f;

// ---- Integrators ----
float int_v = 0.0f;
float int_w = 0.0f;

// ---- Output limits ----
const float UMAX = 1.0f;

// ======== Variabili globali ========
// unsigned long lastValidMocapTime = 0;
float prevX = 0, prevY = 0, prevYawRad = 0;

// Setpoint e input per PID
float setpoint_v = 0, setpoint_w = 0;
float input_v = 0, input_w = 0;
float output_v = 0, output_w = 0;


// ======================= SETUP Control =======================
void setupVelocityPID() {

    
    log("[INIT] Velocity PID initialized");
}

// ======================= FUNZIONI AUSILIARIE =======================
float wrapToPi(float angle) {
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

void updateVelocityEstimator(float dt) {
    static float est_v = 0, est_w = 0;
    const float alpha = 0.2f; // smoothing factor
    
    // Velocità lineare stimata
    float current_speed = sqrt(pow(mocapData.posX - prevX, 2) + 
                              pow(mocapData.posY - prevY, 2)) / dt;
    est_v = alpha * current_speed + (1 - alpha) * est_v;
    
    // Velocità angolare stimata  
    float yaw_rad = radians(mocapData.yaw);
    float deltaYaw = yaw_rad - prevYawRad;
    deltaYaw = wrapToPi(deltaYaw);
    
    est_w = alpha * (deltaYaw / dt) + (1 - alpha) * est_w;
    
    // Aggiorna le variabili globali
    input_v = est_v;
    input_w = est_w;
    
    // Salva i valori attuali per il prossimo ciclo
    prevX = mocapData.posX;
    prevY = mocapData.posY;
    prevYawRad = yaw_rad;
}

bool checkMocapTimeout() {
    unsigned long now = millis();
    
    if (mocapData.mocapValid) {
        lastValidMocapTime = now;
        return false;
    }
    
    if ((now - lastValidMocapTime) > 1500) {
        log("[WARN] MoCap timeout - switching to SAFETY mode");
        currentControlMode = MODE_SAFETY;
        return true;
    }
    
    return false;
}

// ======================= OUTER LOOP - GUIDANCE =======================
void calculateGuidance(float dt, float& v_ref, float& w_ref) {
    float x = mocapData.posX;
    float y = mocapData.posY;
    float yaw_deg = mocapData.yaw;
    
    // Errori di posizione
    float dx = x - hoverX;
    float dy = y - hoverY;
    
    // Conversione a radianti
    float yaw_rad = radians(yaw_deg);
    float hoverYaw_rad = radians(hoverYaw);
    
    // Errore lungo-track (asse longitudinale del veicolo)
    float cos_yaw = cos(yaw_rad);
    float sin_yaw = sin(yaw_rad);
    float e_s = cos_yaw * dx + sin_yaw * dy;
    
    // Errore cross-track (asse trasversale del veicolo)
    float e_c = -sin_yaw * dx + cos_yaw * dy;
    
    // Calcolo riferimento velocità
    const float K_s = 1.0f;  // Guadagno per errore lungo-track
    v_ref = K_s * e_s;
    v_ref = constrain(v_ref, 0.1f, 2.0f);
    
    // Calcolo riferimento velocità angolare
    const float K_psi = 1.0f;  // Guadagno per errore heading
    float e_psi = wrapToPi(hoverYaw_rad - yaw_rad);
    w_ref = K_psi * e_psi;

}

// ======================= INNER LOOP - VELOCITY PID =======================
void applyVelocityPID(float dt) {
  // Calcola le velocità stimate
  updateVelocityEstimator(dt);
    
  // --- PI control ---
  float output_v = computePI(setpoint_v, input_v, int_v, Kp_v_hover, Ki_v, dt);
  float output_w = computePI(setpoint_w, input_w, int_w, Kp_w_hover, Ki_w, dt);

  // Conversione in comandi motore
  float vL = output_v - (output_w * L / 2.0f);
  float vR = output_v + (output_w * L / 2.0f);
        
  vL = constrain(vL, -1.0f, 1.0f);
  vR = constrain(vR, -1.0f, 1.0f);
        
  // DEBUG
  static unsigned long lastMotorDebug = 0;
  unsigned long now = millis();
  if (now - lastMotorDebug > 200) {
      log("[INNER PI] vL: " + String(vL, 3) + ", vR: " + String(vR, 3));
      lastMotorDebug = now;
  }
        
  setLeftMotorSpeed(vL);
  setRightMotorSpeed(vR);
}

// ======================= FUNZIONE PRINCIPALE =======================
void applyHoverControl() {
  unsigned long now = millis();
  static unsigned long lastTime = 0;
  float dt = (now - lastTime) / 1000.0f;
  static bool firstRun = true;
  if (firstRun) {
      lastTime = now;
      firstRun = false;
      return;
  }
    
  // Controllo validità dt
  if (dt <= 0.0f || dt > 1.0f) {
      dt = 0.01f;
  }
  
  // 1. Controllo timeout MoCap
  if (checkMocapTimeout()) {
      return;
  }
  
  // 2. Calcolo guidance (outer loop)
  float v_ref = 0, w_ref = 0;
  calculateGuidance(dt, v_ref, w_ref);
  
  // 3. Imposta setpoint per i PID
  setpoint_v = v_ref;
  setpoint_w = w_ref;
  
  // 4. Esegui controllo PID (inner loop)
  applyVelocityPID(dt);
    // 5. Controlli separati per quota 
  setupVerticalControl();   // Controllo quota
}

float computePI(
    float setpoint,
    float measurement,
    float &integrator,
    float Kp,
    float Ki,
    float dt
) {
    float error = setpoint - measurement;

    // Integratore
    integrator += error * dt;
    integrator = constrain(integrator, -UMAX / Ki, UMAX / Ki);

    // Output
    float u = Kp * error + Ki * integrator;
    u = constrain(u, -UMAX, UMAX);
    return u;
}
