bool initializeWiFi() {
  // Check for the WiFi module
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    return false;
  }

  // WiFi.noLowPowerMode();
  Serial.println("\nInitializing WiFi connection...");

  Serial.print("Connecting to SSID: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  // Try for about 15 seconds
  uint8_t attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 15) {
    delay(1000);
    Serial.print("Attempt ");
    Serial.print(attempts + 1);
    Serial.print("/15, Status: ");
    printConnectionStatus(WiFi.status());
    
    // Optional: Reattempt connection
    WiFi.begin(ssid, password);
    
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\n=== WiFi Connection Failed ===");
      printWifiStatus();
      return false;  // Now correctly returns false if the connection fails
    }

  Serial.println("\n=== WiFi Connected ===");
  printWifiStatus();
  return true; // Return true only if connected successfully
}

void printWifiStatus() {
  Serial.println("=== Network Status ===");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());  
  Serial.print("Signal strength (RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  Serial.println("=====================");
}

void printConnectionStatus(int status) {
  switch(status) {
    case WL_IDLE_STATUS:     Serial.println("Idle"); break;
    case WL_NO_SSID_AVAIL:   Serial.println("No SSID Available"); break;
    case WL_SCAN_COMPLETED:  Serial.println("Scan Completed"); break;
    case WL_CONNECTED:       Serial.println("Connected"); break;
    case WL_CONNECT_FAILED:  Serial.println("Connect Failed"); break;
    case WL_CONNECTION_LOST: Serial.println("Connection Lost"); break;
    case WL_DISCONNECTED:    Serial.println("Disconnected"); break;
    default:                 Serial.println("Unknown Status"); break;
  }
}

bool isWiFiConnected() {
  return WiFi.status() == WL_CONNECTED;
}





