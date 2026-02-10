// #define M 10

// struct MovingAverage {
//   float buffer[M];
//   float sum = 0;
//   int index = 0;
//   int count = 0;
//   bool filled = false;
// };


// float updateMovingAverage(MovingAverage &ma, float v_raw) {
//   ma.sum -= ma.buffer[ma.index]; // Rimuove il valore che stiamo per sovrascrivere
//   ma.buffer[ma.index] = v_raw;   // Inserisce il nuovo valore
//   ma.sum += v_raw;               // Aggiorna la somma

//   ma.index = (ma.index + 1) % M; // Avanza l'indice circolare

//   // Conta i campioni durante lo startup
//   if (!ma.filled) {
//     ma.count++;
//     if (ma.count >= M) {
//       ma.filled = true;
//     }
//   }

//   // Calcola la media
//   if (ma.filled) {
//     return ma.sum / M;
//   } else {
//     return ma.sum / ma.count;
//   }
// }


// MovingAverage filter;

//  float v_hat = updateMovingAverage(filter, v_raw);

