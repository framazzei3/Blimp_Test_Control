// Funzione di inizializzazione del sensore
void initializeToF() {
  sensor_vl53l4cx_sat.begin();
  sensor_vl53l4cx_sat.VL53L4CX_Off();
  delay(100);
  sensor_vl53l4cx_sat.InitSensor(0x12); // Inizializza con configurazione di default
  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();
  Serial.println("ToF initialized successfully");
}

// // Funzione di lettura dati dal VL53L4CX 
// void readVL53L4CX() {
//   VL53L4CX_MultiRangingData_t MultiRangingData;
//   VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
//   uint8_t NewDataReady = 0;
//   int status = 0;
//   int no_of_object_found = 0;

//   // Attendi che i dati siano pronti
//   do {
//     status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
//     delay(5);  // Evita loop troppo rapido
//   } while (!NewDataReady && status == 0);
  
//     if (NewDataReady && status == 0) {
      
//       status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
//       no_of_object_found = pMultiRangingData->NumberOfObjectsFound;

//     // DEBUG PRINT (commentato)
//     // Serial.print("Count=");
//     // Serial.print(pMultiRangingData->StreamCount);
//     // Serial.print(", Objects Detected=");
//     // Serial.println(no_of_object_found);

//     // for (int j = 0; j < no_of_object_found; j++) {
//     //   Serial.print("Object ");
//     //   Serial.print(j);
//     //   Serial.print(": Status=");
//     //   Serial.print(pMultiRangingData->RangeData[j].RangeStatus);
//     //   Serial.print(", Distance=");
//     //   Serial.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
//     //   Serial.print(" mm, Signal=");
//     //   Serial.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps / 65536.0);
//     //   Serial.print(" Mcps, Ambient=");
//     //   Serial.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps / 65536.0);
//     //   Serial.println(" Mcps");
//     // }

//     // blocco per calcolare la quota
//     if (no_of_object_found > 0 && 
//         pMultiRangingData->RangeData[0].RangeStatus == 0) { // prendo il primo punto 0 perche ???
        
//     tofAltitude_m = pMultiRangingData->RangeData[0].RangeMilliMeter / 1000.0;
//     tofAvailable = true;
    
//     } else {
//       tofAvailable = false;
//     }

//     // Pulisce l’interrupt e avvia nuova misura
//     sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
//   }
// }

// Funzione di lettura dati dal VL53L4CX (non bloccante e leggera)
void readVL53L4CX() {
  static unsigned long lastCheck = 0;
  const unsigned long checkInterval = 10; // ms tra un controllo e l'altro

  // Esegui il controllo solo ogni 10 ms
  if (millis() - lastCheck < checkInterval) return;
  
  lastCheck = millis();

  uint8_t NewDataReady = 0;
  // Chiedo al sensore se ha dati pronti
  int status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);

  // status = 0 -> nessun errore comunicazione I2C;
  // NewDataReady = 1 -> sì, ho un nuovo dato pronto;
  // NewDataReady = 0 -> no, sto ancora misurando.
  
  // Se c’è stato un errore nel polling, esci subito
  if (status != 0) return;

  // Se i dati sono pronti, leggi il risultato
  if (NewDataReady) {
    VL53L4CX_MultiRangingData_t MultiRangingData;
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(&MultiRangingData);

  // scarica dalla memoria interna del sensore tutti i dati dell’ultima misura;
  // questi dati sono messi nella struttura MultiRangingData, che contiene:
  // NumberOfObjectsFound -> quanti target ha visto il sensore;
  // RangeData[j].RangeMilliMeter → distanza del target j (in mm);
  // RangeData[j].RangeStatus → stato (0 = misura valida, ≠0 = errore).

    if (status == 0 && MultiRangingData.NumberOfObjectsFound > 0 &&
        MultiRangingData.RangeData[0].RangeStatus == 0) {
      // Usa il primo target valido (tipicamente quello più vicino)
      tofAltitude_m = MultiRangingData.RangeData[0].RangeMilliMeter / 1000.0;
      tofAvailable = true;
    } else {
      tofAvailable = false;
    }

    // Avvia una nuova misura
    sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
  }
}





