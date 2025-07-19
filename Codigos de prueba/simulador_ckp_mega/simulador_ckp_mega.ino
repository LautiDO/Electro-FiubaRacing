const int pinOut = 9;           // Pin de salida conectado a la señal
const int numTeeth = 24;        // Número total de dientes
const int missingTooth = 4;     // Número de diente faltante (índice desde 0)
const unsigned long pulseWidth = 7000;  // Ancho del pulso en microsegundos

void setup() {
  pinMode(pinOut, OUTPUT);
  digitalWrite(pinOut, LOW);    // Asegurarse de que el pin comience en LOW
}

void loop() {
  for (int i = 0; i < numTeeth; i++) {
    if (i == missingTooth || i == missingTooth + 1) {
      // Simula los dos dientes faltantes (común en ruedas de 36-2)
      delayMicroseconds(2 * pulseWidth);
    } else {
      // Genera un "diente"
      digitalWrite(pinOut, HIGH);
      delayMicroseconds(pulseWidth);
      digitalWrite(pinOut, LOW);
      delayMicroseconds(pulseWidth);
    }
  }
}
