//==================================================
//                 VELOCITY ESTIMATION PARAMETERS 
//==================================================
float z_prev = 0.0f;
float v_raw  = 0.0f;
float v_hat  = 0.0f;
float z_meas = 0.0f;

float dt = 0.00f;
bool firstRun = true;
float int_ev = 0.0f;

// Low-pass filter factor (0.8–0.95 typical)
const float alpha_v = 0.8f;

// Dirty derivative / Filtered derivative time constant
const float T = 0.05f;  // [s], tipico 0.03 - 0.1
float alpha = 0.0f;     // sarà aggiornato nel loop in base a dt

// ======================
// Moving Average Parameters
// ======================
const int M = 5;                // finestra media mobile
float v_buffer[M] = {0};   // inizializzo tutti a zero
float bufferSum = 0.0f;          // somma dei campioni
int bufferIndex = 0;             // indice corrente
bool bufferFilled = false;       // diventa true quando il buffer è pieno


//==================================================
//                 CONTROL LIMITS                    
//==================================================
const float U_MIN = -1.0f;
const float U_MAX =  1.0f;
const float V_MAX =  1.0f;   // max vertical speed [m/s]

//==================================================
//                 GAIN SCHEDULING PARAMETERS                    
//==================================================
float v_th = 0.05f;
bool is_up = true;

//==================================================
//              VELOCITY PI CONTROLLER               
//==================================================
// PID variables (velocity loop) - 
float v_ref    = 0.0f;

// PI gains (UP / DOWN)
float Kp_up   = 0.8f;
float Ki_up   = 0.1f;
float Kp_down = 3.1f;
float Ki_down = 1.5f;
float deltaz = 0.0f;
//==================================================
//               ALTITUDE CONTROL                   
//==================================================
float z_ref = 0.5f;   // target altitude [m]
float Kp_z  = 0.6f;   // altitude -> velocity gain


unsigned long lastTimeVertical = 0;
float u_intVertical = 0.0f;
float u_pi = 0.0f;   // comando totale PI 


//==================================================
//                  SETUP                         
//==================================================

void setupVerticalControl() {
  lastTimeVertical = millis();
  log("[INIT] Vertical control initialized");
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

  static unsigned long lastExec = 0;
  unsigned long now = millis();

  if (now - lastExec < 100) return; // 50 ms = 20 Hz
  dt = (now - lastExec) * 1e-3f;
  lastExec = now;

  // Aggiorno alpha in base a dt e T
  alpha = dt / (T + dt);  

  // --- Acquire altitude from mocap ---
  // float z_meas = mocapData.posZ;
  z_meas = tofAltitude_m;

  // ======================
  // VELOCITY ESTIMATION
  // ======================
  if (firstRun) {
    // Prime inizializzazioni
    z_prev = z_meas;
    v_raw = 0.0f;

    // Inizializza buffer a zero
    for (int i = 0; i < M; i++)
      v_buffer[i] = 0.0f;

    bufferSum = 0.0f;
    v_hat = 0.0f;

    firstRun = false;
  } else {

    // Raw derivative
    v_raw = (z_meas - z_prev) / dt;

    // ======================
    // Metodo: Dirty Derivative / Filtered Derivative
    // ======================
    // alpha = dt / (T + dt)
    // v_hat = alpha * v_raw + (1.0f - alpha) * v_hat;

    // ======================
    // MEDIA MOBILE SU v_raw
    // ======================

    // Rimuovo il valore che sto per sovrascrivere
    bufferSum -= v_buffer[bufferIndex];

    // Inserisco il nuovo valore
    v_buffer[bufferIndex] = v_raw;

    // Aggiorno la somma
    bufferSum += v_raw;

    // Avanzo l'indice circolare
    bufferIndex = (bufferIndex + 1) % M;

    // Se ho già riempito il buffer, uso M
    if (bufferIndex == 0)
        bufferFilled = true;

    if (bufferFilled) {
        v_hat = bufferSum / M;           // media piena
    } else {
        v_hat = bufferSum / bufferIndex; // media parziale durante startup
    }

    // ======================
    // Aggiorno z_prev per il prossimo giro
    // ======================
    z_prev = z_meas;

  }

  // ======================
  // ALTITUDE -> VELOCITY (OUTER LOOP)
  // ======================
  v_ref = Kp_z * (z_ref - z_meas);
  v_ref = constrain(v_ref, -V_MAX, V_MAX);

  // ======================
  // GAIN SCHEDULING (qui a ogni ciclo cambio i K anche se la condizione rimane uguale)
  // ======================
  // float Kp_v = (v_ref >= 0.0f) ? Kp_up : Kp_down;
  // float Ki_v = (v_ref >= 0.0f) ? Ki_up : Ki_down;


    // if (is_up && v_ref < -v_th) {
    //     is_up = false;
    // }
    // else if (!is_up && v_ref > v_th) {
    //     is_up = true;
    // }

    // float Kp_v = is_up ? Kp_up : Kp_down;
    // float Ki_v = is_up ? Ki_up : Ki_down;
    
    float Kp_v = Kp_up;
    float Ki_v = Ki_up;


  // ======================
  // VELOCITY CONTROL (INNER LOOP)
  // ======================

  float e_v = v_ref - v_hat;         // errore velocità
  // u_intVertical  += Ki_v * dt * e_v; // integratore vecchio sbagliato perche quando cambi k mantiene gli errori precedenti
  int_ev += e_v * dt;

  // u_pi = Kp_v * e_v + u_intVertical ; // comando totale
  u_pi = Kp_v * e_v + Ki_v * int_ev;


  // saturation + anti-windup
  if (u_pi > U_MAX) {                    
      u_pi = U_MAX;                       // clamp the command to maximum
      int_ev -= e_v * dt;   // undo last integral contribution (anti-windup)
  } else if (u_pi < U_MIN) {       
      u_pi = U_MIN;                       // clamp the command to minimum
      int_ev -= e_v * dt;   // undo last integral contribution (anti-windup)
  }

  // comando motore verticale
  setVerticalMotorSpeed(u_pi);
}


void logVerticalControl() {
  if (currentControlMode != MODE_ALTITUDE_HOLD) return;

  String msg = "[VERT] ";

  msg += "z="     + String(z_meas, 3) + ", ";
  msg += "v_raw=" + String(v_raw, 3)  + ", ";
  msg += "v_hat=" + String(v_hat, 3)  + ", ";
  msg += "v_ref=" + String(v_ref, 3)  + ", ";
  msg += "e_v="   + String(v_ref - v_hat, 3) + ", ";
  msg += "int="   + String(int_ev, 3) + ", ";
  msg += "u="     + String(u_pi, 3);

  log(msg);
}


