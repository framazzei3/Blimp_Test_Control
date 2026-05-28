// ============================================================
//  HOVER CONTROL  —  versione pulita
//  Rimosso: K_s (non usato in fase 3), BRAKE_MARGIN, B_X_MINUS,
//           fase 4 (frenata anticipata), V_MAX, V_REV_MAX (rimane
//           solo V_CRUISE come limite velocità)
//  Aggiunto: isteresi YAW (evita chattering FWD↔YAW)
// ============================================================

// ======== Riferimenti waypoint ========
float hoverX   = 0.0f;   // m
float hoverY   = 0.0f;   // m
float hoverZ   = 1.0f;   // m
float hoverYaw = 0.0f;   // deg

// ======== Riferimenti velocità (output guidance, input PI) ========
float v_hover_ref = 0.0f;
float w_hover_ref = 0.0f;

// ======== Comandi motori ========
float cmd_left  = 0.0f;
float cmd_right = 0.0f;

// ======== Stime velocità ========
float v_hover_est  = 0.0f;
float w_hover_est  = 0.0f;
float v_body_y_est = 0.0f;

// ======== Filtri Butterworth 1° ordine ========
auto mocap_filter_v  = butter<1>(f_n);
auto mocap_filter_w  = butter<1>(f_n);
auto mocap_filter_vy = butter<1>(f_n);

// ======== Soglie geometriche ========
const float ACCEPTANCE_RADIUS  = 0.30f;   // m  — banda morta waypoint
const float REVERSE_THRESHOLD  = 2.62f;   // rad (~150°) — entra in retromarcia
const float YAW_THRESHOLD_IN   = 0.6f;    // rad (~57°)  — entra in fase YAW
const float YAW_THRESHOLD_OUT  = 0.3f;    // rad (~23°)  — esce da fase YAW (isteresi)

// ======== Limiti velocità ========
const float V_MAX_FWD  = 0.20f;  // cap duro — non supera mai questo
const float V_REV_MAX  = 0.25f;   // m/s — velocità massima retromarcia


// ======== Guadagni outer loop ========
float K_s        = 0.10f;   // 
float K_psi      = 0.50f;   // proporzionale errore yaw → w_ref
// float K_sideslip = 0.15f;   // compensazione sideslip laterale → w_ref

// ======== Guadagni PI inner loop ========
float Kp_v_hover = 3.5f;
float Ki_v_hover = 1.0f;

float Kp_w_hover = 2.5f;
float Ki_w_hover = 0.5f;

// float K_s   = 0.07f;   // era 0.05 — avanza un po' più deciso
// float K_psi = 0.10f;   // era 0.08 — corregge yaw più rapidamente

// ======== Integratori PI ========
float int_v = 0.0f;
float int_w = 0.0f;

// ======== Timing ========
unsigned long lastExecHover = 0;
float dt_hover = 0.0f;

// ======== Stato stimatore ========
float prevX      = 0.0f;
float prevY      = 0.0f;
float prevYawRad = 0.0f;

float dist_xy  = 0.0f;   // distanza XY dal target — aggiornata in calculateGuidance

// ======== Stato guidance ========
// 0=hold, 1=reverse, 2=yaw, 3=forward
int  guidancePhase = 0;
bool inYawPhase    = false;   // stato isteresi YAW

unsigned long mocapLostTime = 0;


// ============================================================
//  SETUP
// ============================================================
void setupHoverControl() {

    log("[INIT] Hover control initialized");

    int_v = 0.0f;
    int_w = 0.0f;

    prevX      = mocapData.posX;
    prevY      = mocapData.posY;
    prevYawRad = radians(mocapData.yaw);

    v_hover_est  = 0.0f;
    w_hover_est  = 0.0f;
    v_body_y_est = 0.0f;

    v_hover_ref = 0.0f;
    w_hover_ref = 0.0f;

    guidancePhase = 0;
    inYawPhase    = false;

    mocap_filter_v  = butter<1>(f_n);
    mocap_filter_w  = butter<1>(f_n);
    mocap_filter_vy = butter<1>(f_n);

    lastExecHover = 0;

    if (sdLoggingEnabled && sdCardInitialized) {
        String folderPath = "/fs/" + getDateFolder();

        if (currentControlMode == MODE_HOVER) {
            filenameHover = folderPath + "/hover_" + getDateTimeFilename();
            FILE* file = fopen(filenameHover.c_str(), "w");
            if (file) {
                writeCSVHeaderHover(file);
                fclose(file);
                log("[SD] Hover log created: " + filenameHover);
            } else {
                log("[ERROR] Failed to create hover CSV");
            }
        } else {
            filenameHover = folderPath + "/mission_" + getDateTimeFilename();
            FILE* file = fopen(filenameHover.c_str(), "w");
            if (file) {
                writeCSVHeaderMission(file);
                fclose(file);
                log("[SD] Mission log created: " + filenameHover);
            } else {
                log("[ERROR] Failed to create mission CSV");
            }
        }
    }
}


// ============================================================
//  UTILITY
// ============================================================
float wrapToPi(float angle) {
    while (angle >  PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

// ============================================================
//  STIMATORE VELOCITÀ  (da MoCap differenziato + filtro)
// ============================================================
void updateVelocityEstimator() {

    float dx = mocapData.posX - prevX;
    float dy = mocapData.posY - prevY;

    float vx = dx / dt_hover;
    float vy = dy / dt_hover;

    float yaw_rad = radians(mocapData.yaw);
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    // Proiezione nel frame body
    float v_body_x =  cos_yaw * vx + sin_yaw * vy;   // longitudinale
    float v_body_y = -sin_yaw * vx + cos_yaw * vy;   // laterale (sideslip)

    // Velocità angolare da differenza yaw
    float deltaYaw = wrapToPi(yaw_rad - prevYawRad);
    float om_raw   = deltaYaw / dt_hover;

    // Filtraggio
    v_hover_est  = mocap_filter_v(v_body_x);
    w_hover_est  = mocap_filter_w(om_raw);
    v_body_y_est = mocap_filter_vy(v_body_y);

    prevX      = mocapData.posX;
    prevY      = mocapData.posY;
    prevYawRad = yaw_rad;
}

// ============================================================
//  GUIDANCE  (outer loop)
// ============================================================
void calculateGuidance() {

    float dx   = hoverX - mocapData.posX;
    float dy   = hoverY - mocapData.posY;
    // float dist = sqrtf(dx * dx + dy * dy);
    dist_xy  = sqrtf(dx * dx + dy * dy);   

    float yaw_rad    = radians(mocapData.yaw);
    float desiredYaw = atan2f(dy, dx);
    float e_psi      = wrapToPi(desiredYaw - yaw_rad);

    // ----------------------------------------------------------
    // FASE 0: Banda di accettazione — solo in HOVER_HOLD
    // ----------------------------------------------------------
    if (dist_xy < ACCEPTANCE_RADIUS && hoverMode == HOVER_HOLD) {
        guidancePhase = 0;
        inYawPhase    = false;
        v_hover_ref   = 0.0f;
        int_v         = 0.0f;

        float desiredYawRad = radians(hoverYaw);
        float e_psi_hold    = wrapToPi(desiredYawRad - yaw_rad);
        w_hover_ref = (fabsf(e_psi_hold) < 0.1f) ? 0.0f : K_psi * e_psi_hold;
        if (fabsf(e_psi_hold) < 0.1f) int_w = 0.0f;
        return;
    }

    // ----------------------------------------------------------
    // FASE 1: Waypoint esattamente dietro → retromarcia
    //   K_s usato SOLO qui
    // ----------------------------------------------------------
    if (fabsf(e_psi) > REVERSE_THRESHOLD) {
        guidancePhase = 1;
        inYawPhase    = false;
        v_hover_ref   = constrain(-K_s * dist_xy, -V_REV_MAX, 0.0f);
        w_hover_ref   = 0.0f;
        int_w         = 0.0f;
        return;
    }

    // ----------------------------------------------------------
    // FASE 2: Disallineato → fermati e ruota (con isteresi)
    //   entra se |e_psi| > 20°
    //   esce  se |e_psi| < 10°
    // ----------------------------------------------------------
    // if (!inYawPhase && fabsf(e_psi) > YAW_THRESHOLD_IN)  inYawPhase = true;
    // if ( inYawPhase && fabsf(e_psi) < YAW_THRESHOLD_OUT) inYawPhase = false;

    // if (inYawPhase) {
    //     guidancePhase = 2;

    //     if (fabsf(v_hover_est) > 0.03f) {
    //         // 2a — ancora in moto: frena prima di ruotare
    //         float brake_sign = (v_hover_est > 0.0f) ? -1.0f : 1.0f;
    //         v_hover_ref = brake_sign * 0.08f;
    //         w_hover_ref = 0.0f;
    //     } else {
    //         // 2b — fermo: ruota verso il target
    //         v_hover_ref = 0.0f;
    //         int_v       = 0.0f;
    //         w_hover_ref = constrain(K_psi * e_psi, -W_MAX, W_MAX);
    //     }
    //     return;
    // }

    // ----------------------------------------------------------
    // FASE 3: Allineato → avanza con profilo trapezoidale
    //   - crociera fissa a V_CRUISE quando dist > SLOW_DIST
    //   - rampa lineare verso 0 quando dist < SLOW_DIST
    //   - align_factor riduce v_ref se c'è ancora errore yaw residuo
    // ----------------------------------------------------------
    guidancePhase = 3;
    // float align_factor = cosf(e_psi);

    float v_target = constrain(K_s * dist_xy, 0.0f, V_MAX_FWD);
    
    // v_hover_ref = v_target * align_factor;
    // v_hover_ref = v_target;   // avanza sempre, yaw gestisce l'orientamento separatamente

    float align_factor = fmaxf(0.0f, cosf(e_psi));   // clamp: [0, 1]
    v_hover_ref = v_target * align_factor;

    w_hover_ref = constrain(
        K_psi * e_psi, //  - K_sideslip * v_body_y_est,
        -W_MAX, W_MAX
    );
}

// ============================================================
//  VELOCITY PI  (inner loop)
// ============================================================
void applyVelocityPID() {

    float u, v;

    // if (guidancePhase == 2) {
    //     // In fase YAW: congela integratore v e forza u=0
    //     // evita cross-coupling rotazione ↔ oscillazione longitudinale
    //     int_v = 0.0f;
    //     u = 0.0f;
    //     v = computePI(w_hover_ref, w_hover_est, int_w, Kp_w_hover, Ki_w_hover, dt_hover);
    // } else {
    u = computePI(v_hover_ref, v_hover_est, int_v, Kp_v_hover, Ki_v_hover, dt_hover);
    v = computePI(w_hover_ref, w_hover_est, int_w, Kp_w_hover, Ki_w_hover, dt_hover);
    // }

    // cmd_left  = (u - v) / 2.0f;
    // cmd_right = (u + v) / 2.0f;

    // ---- Mixing con compensazione asimmetrica su v (yaw) ----
    // Il motore che gira al contrario è meno efficiente → moltiplica per COMP_FACTOR
    float v_fwd = v;                    // contributo yaw sul motore in avanti
    float v_bwd = v * COMP_FACTOR;      // contributo yaw sul motore all'indietro (compensato)

    // cmd = u (lineare, simmetrico) ± v (yaw, asimmetrico)
    // Convenzione: v > 0 → gira a destra → right avanti, left indietro
    if (v >= 0.0f) {
        cmd_left  = (u - v_bwd) / 2.0f;   // left indietro → compensato
        cmd_right = (u + v_fwd) / 2.0f;   // right avanti  → nominale
    } else {
        cmd_left  = (u - v_fwd) / 2.0f;   // left avanti   → nominale
        cmd_right = (u + v_bwd) / 2.0f;   // right indietro → compensato
    }

    // Saturazione
    cmd_left  = constrain(cmd_left,  -1.0f, 1.0f);
    cmd_right = constrain(cmd_right, -1.0f, 1.0f);

    // Anti-windup: se entrambi i comandi sono sotto soglia fisica
    // il motore non attuera' → annulla il contributo integrale appena aggiunto
    float err_v = v_hover_ref - v_hover_est;
    float err_w = w_hover_ref - w_hover_est;

    // Anti-windup SOLO per saturazione reale di u/v
    // NON per deadband del mixing — in quel caso l'integratore deve continuare a crescere
    if (fabsf(u) >= 1.0f) {
        if (err_v * int_v > 0.0f) int_v -= err_v * dt_hover;
    }
    if (fabsf(v) >= 1.0f) {
        if (err_w * int_w > 0.0f) int_w -= err_w * dt_hover;
    }

    // Azzeramento fisico: sotto soglia manda 0 al motore
    if (fabsf(cmd_left)  < MOTOR_THRESHOLD) cmd_left  = 0.0f;
    if (fabsf(cmd_right) < MOTOR_THRESHOLD) cmd_right = 0.0f;

    setLeftMotorSpeed(cmd_left);
    setRightMotorSpeed(cmd_right);
}

// ============================================================
//  MAIN CONTROL LOOP
// ============================================================
void applyHoverControl() {

    // Watchdog MoCap
    if (!mocapData.mocapValid) {
        if (mocapLostTime == 0) mocapLostTime = millis();
        if (millis() - mocapLostTime > 3000) {
            log("[WARN] MoCap lost >3s -> SAFETY");
            currentControlMode = MODE_SAFETY;
        }
        return;
    } else {
        mocapLostTime = 0;
    }

    unsigned long now = millis();

    if (lastExecHover == 0) {
        lastExecHover = now;
        return;
    }

    if (now - lastExecHover < 50) return;   // 20 Hz

    dt_hover      = (now - lastExecHover) * 1e-3f;
    lastExecHover = now;

    updateVelocityEstimator();
    calculateGuidance();
    applyVelocityPID();

    logHoverControl();
    
    // Log CSV solo in modalità HOVER (waypoint ha il suo logCSVMissionRow)
    if (currentControlMode == MODE_HOVER) {
        logCSVHoverRow();
    }

}

// ============================================================
//  PI CONTROLLER
// ============================================================
float computePI(
    float  setpoint,
    float  measurement,
    float &integrator,
    float  Kp,
    float  Ki,
    float  dt
    ) {
        float error = setpoint - measurement;

        integrator += error * dt;

        float u = Kp * error + Ki * integrator;

        // Saturazione con anti-windup: annulla l'ultimo contributo se saturo
        if (u > 1.0f) {
            u = 1.0f;
            integrator -= error * dt;
        } else if (u < -1.0f) {
            u = -1.0f;
            integrator -= error * dt;
        }
    return u;
}

// ============================================================
//  LOGGING
// ============================================================
void logHoverControl() {
    // float dx   = hoverX - mocapData.posX;
    // float dy   = hoverY - mocapData.posY;
    // float dist = sqrtf(dx * dx + dy * dy);

    String msg = "[HOVER] ";
    msg += "phase=" + String(guidancePhase)           + " | ";
    msg += "dist="  + String(dist_xy, 2)                 + "m | ";
    msg += "v_ref=" + String(v_hover_ref, 3) + " v_est=" + String(v_hover_est, 3) + " | ";
    msg += "w_ref=" + String(w_hover_ref, 3) + " w_est=" + String(w_hover_est, 3) + " | ";
    msg += "vy="    + String(v_body_y_est, 3);
    log(msg);
}

void writeCSVHeaderMission(FILE* file) {
    fprintf(file,
        "millis,time,mode,"
        "bat_voltage,bat_percent,"
        "wp_index,wp_count,wp_x,wp_y,wp_z,wp_yaw,"
        "mocap_x,mocap_y,mocap_z,mocap_yaw,mocap_valid,"
        "dist_xy,dist_3d,guidance_phase,"
        "v_hover_ref,w_hover_ref,"
        "v_hover_est,w_hover_est,v_body_y_est,"
        "int_v,int_w,"
        "cmd_left,cmd_right,"
        "z_ref,z_meas,tof_alt,"
        "v_raw,v_hat_butter,"
        "v_ref_unsat,v_ref,"
        "e_v,int_ev,de_v_dt,de_v_dt_filt,"
        "uz_pid,"
        "Kp_v,Ki_v,Kd_v,Kp_z,"
        "Kp_v_hover,Ki_v_hover,Kp_w_hover,Ki_w_hover,"
        "K_s,K_psi,"
        "dt_hover,dt_vert\n"
    );
}

void writeCSVHeaderHover(FILE* file) {
    fprintf(file,
        "millis,time,mode,"
        "bat_voltage,bat_percent,"
        "hover_x,hover_y,hover_z,hover_yaw,"
        "mocap_x,mocap_y,mocap_z,mocap_yaw,mocap_valid,"
        "dist_xy,guidance_phase,"
        "v_hover_ref,w_hover_ref,"
        "v_hover_est,w_hover_est,v_body_y_est,"
        "int_v,int_w,"
        "cmd_left,cmd_right,"
        "z_ref,z_meas,tof_alt,"
        "v_raw,v_hat_butter,"
        "v_ref_unsat,v_ref,"
        "e_v,int_ev,de_v_dt,de_v_dt_filt,"
        "uz_pid,"
        "Kp_v,Ki_v,Kd_v,Kp_z,"
        "Kp_v_hover,Ki_v_hover,Kp_w_hover,Ki_w_hover,"
        "K_s,K_psi,"
        "dt_hover,dt_vert\n"
    );
}

void logCSVMissionRow() {
    if (!sdLoggingEnabled || !sdCardInitialized) return;
    if (filenameHover == "") return;

    FILE* file = fopen(filenameHover.c_str(), "a");
    if (!file) return;

    // float dx      = target.x - mocapData.posX;
    // float dy      = target.y - mocapData.posY;
    // float dist_xy = sqrtf(dx * dx + dy * dy);

    fprintf(file, "%lu,%s,%s,",        millis(), getCurrentTimeString().c_str(), getModeName(currentControlMode).c_str());
    fprintf(file, "%.2f,%.1f,",        batteryStatus.voltage, batteryStatus.percent);
    fprintf(file, "%d,%d,%.4f,%.4f,%.4f,%.2f,", currentWaypointIndex, waypointCount, target.x, target.y, target.z, target.yaw);
    fprintf(file, "%.4f,%.4f,%.4f,%.2f,%d,",    mocapData.posX, mocapData.posY, mocapData.posZ, mocapData.yaw, (int)mocapData.mocapValid);
    fprintf(file, "%.4f,%.4f,%d,",     dist_xy, distToTarget, guidancePhase);
    fprintf(file, "%.4f,%.4f,",        v_hover_ref, w_hover_ref);
    fprintf(file, "%.4f,%.4f,%.4f,",   v_hover_est, w_hover_est, v_body_y_est);
    fprintf(file, "%.4f,%.4f,",        int_v, int_w);
    fprintf(file, "%.4f,%.4f,",        cmd_left, cmd_right);
    fprintf(file, "%.4f,%.4f,%.4f,",   z_ref, z_meas, tofAltitude_m);
    fprintf(file, "%.4f,%.4f,",        v_raw, v_hat_butter);
    fprintf(file, "%.4f,%.4f,",        v_ref_unsat, v_ref);
    fprintf(file, "%.4f,%.4f,%.4f,%.4f,", e_v, int_ev, de_v_dt, de_v_dt_filt);
    fprintf(file, "%.4f,",             uz_pid);
    fprintf(file, "%.3f,%.3f,%.3f,%.3f,", Kp_v, Ki_v, Kd_v, Kp_z);
    fprintf(file, "%.3f,%.3f,%.3f,%.3f,", Kp_v_hover, Ki_v_hover, Kp_w_hover, Ki_w_hover);
    fprintf(file, "%.3f,%.3f,",   K_s, K_psi);
    fprintf(file, "%.4f,%.4f\n",       dt_hover, dt_vert);

    fclose(file);
}

void logCSVHoverRow() {
    if (!sdLoggingEnabled || !sdCardInitialized) return;
    if (filenameHover == "") return;

    FILE* file = fopen(filenameHover.c_str(), "a");
    if (!file) return;

    // float dx      = hoverX - mocapData.posX;
    // float dy      = hoverY - mocapData.posY;
    // float dist_xy = sqrtf(dx * dx + dy * dy);

    fprintf(file, "%lu,%s,%s,",        millis(), getCurrentTimeString().c_str(), getModeName(currentControlMode).c_str());
    fprintf(file, "%.2f,%.1f,",        batteryStatus.voltage, batteryStatus.percent);
    fprintf(file, "%.4f,%.4f,%.4f,%.2f,", hoverX, hoverY, hoverZ, hoverYaw);
    fprintf(file, "%.4f,%.4f,%.4f,%.2f,%d,", mocapData.posX, mocapData.posY, mocapData.posZ, mocapData.yaw, (int)mocapData.mocapValid);
    fprintf(file, "%.4f,%d,",          dist_xy, guidancePhase);
    fprintf(file, "%.4f,%.4f,",        v_hover_ref, w_hover_ref);
    fprintf(file, "%.4f,%.4f,%.4f,",   v_hover_est, w_hover_est, v_body_y_est);
    fprintf(file, "%.4f,%.4f,",        int_v, int_w);
    fprintf(file, "%.4f,%.4f,",        cmd_left, cmd_right);
    fprintf(file, "%.4f,%.4f,%.4f,",   z_ref, z_meas, tofAltitude_m);
    fprintf(file, "%.4f,%.4f,",        v_raw, v_hat_butter);
    fprintf(file, "%.4f,%.4f,",        v_ref_unsat, v_ref);
    fprintf(file, "%.4f,%.4f,%.4f,%.4f,", e_v, int_ev, de_v_dt, de_v_dt_filt);
    fprintf(file, "%.4f,",             uz_pid);
    fprintf(file, "%.3f,%.3f,%.3f,%.3f,", Kp_v, Ki_v, Kd_v, Kp_z);
    fprintf(file, "%.3f,%.3f,%.3f,%.3f,", Kp_v_hover, Ki_v_hover, Kp_w_hover, Ki_w_hover);
    fprintf(file, "%.3f,%.3f,",        K_s, K_psi);
    fprintf(file, "%.4f,%.4f\n",       dt_hover, dt_vert);

    fclose(file);
}