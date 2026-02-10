void printCurrentDateTime() {
    if (!ntpInitialized) {
        return;
    }
    
    // Get epoch time
    time_t epochTime = ntpClient.getEpochTime();
    
    // Convert to struct tm
    struct tm *ptm = gmtime(&epochTime);
    
    // Format date and time
    char dateTimeString[50];
    strftime(dateTimeString, sizeof(dateTimeString), "%A, %B %d %Y %H:%M:%S", ptm);
    
    String formattedDateTime = String(dateTimeString);
    if (formattedDateTime != "") {
        log("[INFO] Current Date/Time: " + formattedDateTime);
    }
}

String getDateFolder() {
    time_t epochTime = ntpClient.getEpochTime();
    struct tm *ptm = gmtime(&epochTime);
    char buf[11];
    strftime(buf, sizeof(buf), "%Y%m%d", ptm);
    return String(buf);
}

String getDateTimeFilename() {
    time_t epochTime = ntpClient.getEpochTime();
    struct tm *ptm = gmtime(&epochTime);
    char buf[20];
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", ptm);  // Formato: AAAAMMGG_HHMMSS
    return String(buf) + ".csv";
}

String getCurrentTimeString() {
    time_t epochTime = ntpClient.getEpochTime();
    struct tm *ptm = gmtime(&epochTime);
    char buf[20];
    strftime(buf, sizeof(buf), "%H:%M:%S", ptm);
    return String(buf);
}