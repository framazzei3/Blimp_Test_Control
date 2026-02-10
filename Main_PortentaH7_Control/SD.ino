// ========== FUNCTIONS ==========
void initSDCard() {
    log("[INFO]: Mounting SD card...");
    int err = fs.mount(&block_device);

    if (err) {
        log("[WARN]: No filesystem found, formatting...");
        err = fs.reformat(&block_device);
        if (err) {
            log("[ERROR]: Failed to format SD card");
            sdCardInitialized = false;
            return;  // Esce senza valore di ritorno
        }
        // Prova a montare di nuovo dopo la formattazione
        err = fs.mount(&block_device);
        if (err) {
            log("[ERROR]: Failed to mount after formatting");
            sdCardInitialized = false;
            return;
        }
    }

    // Crea la cartella con la data
    String folderPath = "/fs/" + getDateFolder();
    if (mkdir(folderPath.c_str(), 0777) != 0 && errno != EEXIST) {
        log("[ERROR]: Failed to create date folder");
        sdCardInitialized = false;
        return;
    }
    
    sdCardInitialized = true;  // Tutto ok!
}

void writeCSVHeader(FILE* file) {
    fprintf(file,
        "millis,time,mode,"
        "bat_voltage,bat_percent,"
        "dt,"
        "v_raw,v_hat_butter,v_ref,"
        "z_ref,z_meas,"
        "e_v,int_ev,de_v_dt,de_v_dt_filt,uz_pid,"
        "Kp_up,Ki_up,Kd_up,Kp_down,Ki_down,Kd_down,Kp_z,"
        "tof_alt,"
        "leftThrust,rightThrust,verticalThrust\n"
    );

}

void logSensorData() {

    // Esci subito se la SD non è disponibile
    if (!sdLoggingEnabled) return;     
    if (!sdCardInitialized) return;

    static unsigned long lastLogTime = 0;
    const unsigned long logInterval = 100; // Log ogni 100ms


    unsigned long currentTime = millis();
    if (currentTime - lastLogTime < logInterval) return;
    lastLogTime = currentTime;

    // Initialize filename on first call
    if (filename == "") {
        String folderPath = "/fs/" + getDateFolder();
        filename = folderPath + "/" + getDateTimeFilename();
        
        // Create file and write header
        FILE* file = fopen(filename.c_str(), "w");
        if (!file) {
            log("[ERROR] Failed to create CSV file");
            return;
        }
        writeCSVHeader(file);
        fclose(file);
        log("[INFO] Created new log file: " + filename);
    }
    
    // Open file in append mode
    FILE* file = fopen(filename.c_str(), "a");
    if (!file) {
        log("[ERROR] Failed to open CSV file for writing");
        return;
    }

    // ================================
    //       SCRITTURA CSV
    // ================================

    // Timestamp and control mode
    fprintf(file, "%lu,%s,%s,", 
            currentTime, 
            getCurrentTimeString().c_str(), 
            getModeName(currentControlMode).c_str());

    // Battery data
    fprintf(file, "%.2f,%.1f,",
            batteryStatus.voltage,
            batteryStatus.percent);

    // Control timing
    fprintf(file, "%.4f,", dt_vert);

    // Velocity
    fprintf(file, "%.4f,%.4f,%.4f,",
            v_raw,
            v_hat_butter,
            v_ref);

    // Altitude
    fprintf(file, "%.4f,%.4f,",
            z_ref,
            z_meas);

    // PID controller
    fprintf(file, "%.4f,%.4f,%.4f,%.4f,%.4f,",
            e_v,
            int_ev,
            de_v_dt,
            de_v_dt_filt,
            uz_pid);

    // PI gains
    fprintf(file, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,",
            Kp_up,
            Ki_up,
            Kd_up,
            Kp_down,
            Ki_down,
            Kd_down,
            Kp_z);

    // ToF altitude
    fprintf(file, "%.4f,",
            tofAltitude_m);

    // Motor thrusts
    fprintf(file, "%.4f,%.4f,%.4f\n",
            leftThrust,
            rightThrust,
            verticalThrust);
   
    fflush(file);
    fclose(file);

}




